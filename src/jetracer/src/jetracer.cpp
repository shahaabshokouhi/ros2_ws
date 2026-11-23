/*
 * ROS2 C++ node bridging serial communication with JetRacer controller.
 */
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <boost/asio.hpp>
using namespace std::chrono_literals;

// Packet constants
static constexpr uint8_t HEAD1 = 0xAA;
static constexpr uint8_t HEAD2 = 0x55;
static constexpr uint8_t TYPE_VELOCITY    = 0x11;
static constexpr uint8_t TYPE_PARAMS      = 0x12;
static constexpr uint8_t TYPE_COEFFICIENT = 0x13;

// Simple checksum: sum of bytes
static uint8_t checksum(const uint8_t *buf, size_t len) {
  uint8_t sum = 0;
  for(size_t i = 0; i < len; ++i) sum += buf[i];
  return sum;
}

class JetracerBridge : public rclcpp::Node {
public:
  JetracerBridge()
  : Node("jetracer_bridge"), serial_(io_) {
    // Parameters
    port_name_ = this->declare_parameter("port_name", "/dev/ttyACM0");
    publish_tf_ = this->declare_parameter("publish_odom_transform", true);
    agent_name_ = this->declare_parameter("agent_name", "agent_0");

    // Open serial port. If the board is missing, keep running: the reader
    // thread retries every second, and the heartbeat says false meanwhile, so
    // the safety gate and the planner hold the car and say why (lab, 23 Nov:
    // a missing board used to end this process, and nothing reported it).
    std::string err;
    if (openPort(err)) {
      port_ok_ = true;
      ever_open_ = true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "NO MOTOR BOARD: cannot open %s (%s). Check its USB cable and its "
                   "power; retrying every second, the car will not move until it is found",
                   port_name_.c_str(), err.c_str());
    }
    last_rx_ = std::chrono::steady_clock::now();

    // Publishers & TF
    imu_pub_  = this->create_publisher<sensor_msgs::msg::Imu>(agent_name_ + "/imu", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(agent_name_ + "/odom", 10);
    lvel_pub_ = this->create_publisher<std_msgs::msg::Int32>(agent_name_ + "/motor/lvel", 10);
    rvel_pub_ = this->create_publisher<std_msgs::msg::Int32>(agent_name_ + "/motor/rvel", 10);
    lset_pub_ = this->create_publisher<std_msgs::msg::Int32>(agent_name_ + "/motor/lset", 10);
    rset_pub_ = this->create_publisher<std_msgs::msg::Int32>(agent_name_ + "/motor/rset", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    // Heartbeat at 10 Hz: true while the port is open and the board is sending.
    // The SLAM node's safety gate and the exploration planner stop when it is
    // false or stops coming (this process died).
    ok_pub_ = this->create_publisher<std_msgs::msg::Bool>(agent_name_ + "/base_driver/ok", 10);
    ok_timer_ = this->create_wall_timer(100ms, [this] {
      std_msgs::msg::Bool m;
      m.data = port_ok_ && std::chrono::steady_clock::now() - last_rx_.load() < 500ms;
      ok_pub_->publish(m);
    });

    // cmd_vel subscriber
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      agent_name_ + "/cmd_vel", 10,
      std::bind(&JetracerBridge::cmdVelCallback, this, std::placeholders::_1)
    );

    // Start reader thread
    reader_thread_ = std::thread(&JetracerBridge::serialReadLoop, this);

    // Velocity timer at 50Hz
    vel_timer_ = this->create_wall_timer(
      20ms, std::bind(&JetracerBridge::sendVelocity, this)
    );

    last_cmd_time_ = this->now();
  }

  ~JetracerBridge() override {
    running_ = false;                       // the reader polls with a timeout and exits
    if (reader_thread_.joinable()) reader_thread_.join();
    std::lock_guard<std::mutex> lk(serial_mx_);
    if (port_ok_) {                         // a last stop command
      x_ = y_ = yaw_ = 0.0;
      writeVelocity();
    }
    boost::system::error_code ec;
    serial_.close(ec);
  }

private:
  bool openPort(std::string &err) {
    try {
      serial_.open(port_name_);
      serial_.set_option(boost::asio::serial_port_base::baud_rate(115200));
      serial_.set_option(boost::asio::serial_port_base::character_size(8));
      serial_.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));
      serial_.set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one));
      serial_.set_option(boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none));
      return true;
    } catch(const std::exception &e) {
      err = e.what();
      boost::system::error_code ec;
      serial_.close(ec);
      return false;
    }
  }

  // The port failed (cable pulled, board reset): close it and reopen it every
  // second until the device is back. Called by the reader thread only.
  void reconnect() {
    if (ever_open_)
      RCLCPP_ERROR(get_logger(), "Motor board on %s lost: stopped; reopening it every second",
                   port_name_.c_str());
    {
      std::lock_guard<std::mutex> lk(serial_mx_);
      boost::system::error_code ec;
      serial_.close(ec);
    }
    int tries = 0;
    while (running_ && rclcpp::ok()) {
      std::this_thread::sleep_for(1s);
      std::lock_guard<std::mutex> lk(serial_mx_);
      std::string err;
      if (openPort(err)) {
        x_ = y_ = yaw_ = 0.0;               // no old command after the gap
        last_rx_ = std::chrono::steady_clock::now();
        port_ok_ = true;
        if (ever_open_) RCLCPP_WARN(get_logger(), "Motor board on %s is back", port_name_.c_str());
        else RCLCPP_WARN(get_logger(), "Motor board on %s found", port_name_.c_str());
        ever_open_ = true;
        return;
      }
      if (++tries % 10 == 0)
        RCLCPP_ERROR(get_logger(), "still no motor board on %s after %d s (%s)",
                     port_name_.c_str(), tries, err.c_str());
    }
  }

  // Read exactly n bytes; false on a port error or at shutdown.
  bool readExact(uint8_t *buf, size_t n) {
    size_t got = 0;
    while (got < n) {
      if (!running_ || !rclcpp::ok() || !port_ok_) return false;   // port_ok_: a write failed
      pollfd pfd{serial_.native_handle(), POLLIN, 0};
      const int r = ::poll(&pfd, 1, 200);
      if (r == 0) continue;                 // timeout: check running_ again
      if (r < 0 || (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))) {
        port_ok_ = false;
        return false;
      }
      boost::system::error_code ec;
      got += serial_.read_some(boost::asio::buffer(buf + got, n - got), ec);
      if (ec) {
        port_ok_ = false;
        return false;
      }
    }
    return true;
  }

   // In your class definition, under “Members”:
  bool odom_initialized_{false};
  double odom_offset_x_{0.0}, odom_offset_y_{0.0}, odom_offset_yaw_{0.0};
  
  // callback for cmd_vel
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(serial_mx_);
    x_ = msg->linear.x;
    y_ = msg->linear.y;
    yaw_ = msg->angular.z;
    last_cmd_time_ = this->now();
  }

  // send velocity periodically
  void sendVelocity() {
    rclcpp::Time tnow = this->now();
    std::lock_guard<std::mutex> lk(serial_mx_);
    if ((tnow - last_cmd_time_).seconds() > 1.0) {
      x_ = y_ = yaw_ = 0.0;
    }
    if (port_ok_) writeVelocity();
  }

  // Caller holds serial_mx_.
  void writeVelocity() {
    uint8_t buf[11];
    buf[0] = HEAD1; buf[1] = HEAD2; buf[2] = 0x0B; buf[3] = TYPE_VELOCITY;
    int16_t xi = static_cast<int16_t>(x_ * 1000);
    buf[4] = (xi >> 8) & 0xFF; buf[5] = xi & 0xFF;
    int16_t yi = static_cast<int16_t>(y_ * 1000);
    buf[6] = (yi >> 8) & 0xFF; buf[7] = yi & 0xFF;
    int16_t yw = static_cast<int16_t>(yaw_ * 1000);
    buf[8] = (yw >> 8) & 0xFF; buf[9] = yw & 0xFF;
    buf[10] = checksum(buf, 10);
    boost::system::error_code ec;
    boost::asio::write(serial_, boost::asio::buffer(buf, 11), ec);
    if (ec) {                               // e.g. "No such device": the reader reconnects
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Motor board write failed: %s",
                            ec.message().c_str());
      port_ok_ = false;
    }
  }

  // continuous serial reader
  void serialReadLoop() {
    std::vector<uint8_t> packet;
    while (running_ && rclcpp::ok()) {
      if (!port_ok_) {
        reconnect();
        continue;
      }
      uint8_t byte;
      if (!readExact(&byte, 1)) continue;
      if (byte != HEAD1) continue;
      if (!readExact(&byte, 1)) continue;
      if (byte != HEAD2) continue;

      // size
      if (!readExact(&byte, 1)) continue;
      size_t sz = byte;
      if (sz < 42) continue;                // handlePacket reads up to byte 41 (also: noise after a reconnect)
      packet.resize(sz);
      packet[0] = HEAD1;
      packet[1] = HEAD2;
      packet[2] = static_cast<uint8_t>(sz);

      // rest of packet
      if (!readExact(packet.data()+3, sz-3)) continue;

      // checksum
      if (checksum(packet.data(), sz-1) != packet[sz-1]) {
        RCLCPP_WARN(get_logger(), "Checksum mismatch, dropping packet");
        continue;
      }

      last_rx_ = std::chrono::steady_clock::now();
      handlePacket(packet);
    }
  }

  void handlePacket(const std::vector<uint8_t> &data) {
    rclcpp::Time tnow = this->now();

    // Decode IMU
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = tnow;
    imu.header.frame_id = "base_imu_link";
    for(int i=0; i<3; ++i) {
      int16_t raw = (data[4+2*i]<<8)|(data[5+2*i]);
      double val = raw / 32768.0 * (2000.0/180.0*M_PI);
      if(i==0) imu.angular_velocity.x = val;
      if(i==1) imu.angular_velocity.y = val;
      if(i==2) imu.angular_velocity.z = val;
    }
    for(int i=0; i<3; ++i) {
      int16_t raw = (data[10+2*i]<<8)|(data[11+2*i]);
      double val = raw / 32768.0 * 2.0 * 9.8;
      if(i==0) imu.linear_acceleration.x = val;
      if(i==1) imu.linear_acceleration.y = val;
      if(i==2) imu.linear_acceleration.z = val;
    }
    int16_t raw_yaw = (data[20]<<8)|data[21];
    double yaw_deg = raw_yaw / 10.0;
    {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_deg*M_PI/180.0);
    imu.orientation = tf2::toMsg(q);
    }
    imu_pub_->publish(imu);

    // Decode odom
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = tnow;
    odom.header.frame_id = "odom";
    double raw_px = static_cast<int16_t>((data[22]<<8)|data[23]) / 1000.0;
    double raw_py = static_cast<int16_t>((data[24]<<8)|data[25]) / 1000.0;
    double raw_pyaw = static_cast<int16_t>((data[26]<<8)|data[27]) / 1000.0;
    
    if (!odom_initialized_) {
      // Initialize odom offsets
      odom_offset_x_ = raw_px;
      odom_offset_y_ = raw_py;
      odom_offset_yaw_ = raw_pyaw;
      odom_initialized_ = true;
    }

    double px = raw_px - odom_offset_x_;
    double py = raw_py - odom_offset_y_;
    double pyaw = raw_pyaw - odom_offset_yaw_;

    odom.pose.pose.position.x = px;
    odom.pose.pose.position.y = py;
    {
      tf2::Quaternion q2;
      q2.setRPY(0, 0, pyaw);
      odom.pose.pose.orientation = tf2::toMsg(q2);
    }

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = tnow;
      t.header.frame_id = "odom";
      t.child_frame_id = "base_footprint";
      t.transform.translation.x = px;
      t.transform.translation.y = py;
      t.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(t);
    }
    odom_pub_->publish(odom);

    // Decode motors
    for(int i=0; i<4; ++i) {
      int16_t raw = (data[34+2*i]<<8)|(data[35+2*i]);
      std_msgs::msg::Int32 msg; msg.data = raw;
      if(i==0) lvel_pub_->publish(msg);
      if(i==1) rvel_pub_->publish(msg);
      if(i==2) lset_pub_->publish(msg);
      if(i==3) rset_pub_->publish(msg);
    }
  }

  // Members
  boost::asio::io_service io_;
  boost::asio::serial_port serial_;
  std::string port_name_;
  bool publish_tf_;
  std::string agent_name_;

  double x_{0}, y_{0}, yaw_{0};            // guarded by serial_mx_ (written by cmdVelCallback too)
  std::mutex serial_mx_;                    // writes, close and reopen (the reader thread reads without it)
  std::atomic<bool> port_ok_{false};
  std::atomic<bool> ever_open_{false};    // the board was open once (lost vs never found)
  std::atomic<bool> running_{true};
  std::atomic<std::chrono::steady_clock::time_point> last_rx_{std::chrono::steady_clock::time_point{}};
  rclcpp::Time last_cmd_time_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lvel_pub_, rvel_pub_, lset_pub_, rset_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr vel_timer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ok_pub_;
  rclcpp::TimerBase::SharedPtr ok_timer_;
  std::thread reader_thread_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JetracerBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
