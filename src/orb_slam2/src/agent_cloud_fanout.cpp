#include <unordered_map>
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <orbslam2_msgs/msg/map_point.hpp>

class AgentCloudFanout : public rclcpp::Node {
public:
  AgentCloudFanout()
  : Node("agent_cloud_fanout")
  {
    // Params
    frame_id_   = declare_parameter<std::string>("output_frame", "map");
    input_topic_= declare_parameter<std::string>("input_topic", "/orb_slam2/single_mappoint");
    base_topic_ = declare_parameter<std::string>("base_output_topic", "/map_receiver");
    rate_hz_    = declare_parameter<double>("publish_rate_hz", 5.0);
    apply_orb_axes_ = declare_parameter<bool>("apply_orb_axes_transform", false);
    // ORB->RViz axes (example): (x',y',z')=(z,-x,-y)

    // Sub (best_effort matches typical sensor pub)
    sub_ = create_subscription<orbslam2_msgs::msg::MapPoint>(
      input_topic_, rclcpp::QoS(200).best_effort(),
      std::bind(&AgentCloudFanout::onPoint, this, std::placeholders::_1));

    // Timer
    auto period_ms = static_cast<int>(1000.0 / std::max(0.1, rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&AgentCloudFanout::publishAll, this));

    RCLCPP_INFO(get_logger(), "Fanout: sub=%s base_topic=%s frame=%s rate=%.2f Hz axes_transform=%s",
                input_topic_.c_str(), base_topic_.c_str(), frame_id_.c_str(), rate_hz_,
                apply_orb_axes_ ? "ON" : "OFF");
  }

private:
  using PointsById = std::unordered_map<uint64_t, geometry_msgs::msg::Point>;

  // Lazily create (or get) a publisher for an agent
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& getPub(const std::string& agent) {
    auto it = pubs_.find(agent);
    if (it != pubs_.end()) return it->second;

    // Topic: /map_receiver/<agent>/pointcloud
    const std::string topic = base_topic_ + "/" + agent + "/pointcloud";
    auto pub = create_publisher<sensor_msgs::msg::PointCloud2>(topic, rclcpp::SensorDataQoS());
    RCLCPP_INFO(get_logger(), "Created publisher: %s", topic.c_str());
    return pubs_.emplace(agent, pub).first->second;
  }

  void onPoint(const orbslam2_msgs::msg::MapPoint::SharedPtr msg) {
    if (!msg) return;
    if (msg->is_bad) return;

    // Insert if new (per-agent de-dup by id)
    auto& per_agent = store_[msg->agent_name];

    geometry_msgs::msg::Point p = msg->position;
    if (apply_orb_axes_) {
      geometry_msgs::msg::Point q;
      // Example transform used in your sender: (z, -x, -y)
      q.x =  static_cast<double>(msg->position.z);
      q.y = -static_cast<double>(msg->position.x);
      q.z = -static_cast<double>(msg->position.y);
      p = q;
    }
    per_agent.emplace(msg->id, p);

    // Ensure a publisher exists (lazy)
    (void)getPub(msg->agent_name);
  }

  void publishAll() {
    const rclcpp::Time t = now();

    for (auto& agent_pair : store_) {
      const std::string& agent = agent_pair.first;
      PointsById& points = agent_pair.second;

      auto pub_it = pubs_.find(agent);
      if (pub_it == pubs_.end()) continue;
      auto& pub = pub_it->second;

      const size_t N = points.size();
      if (N == 0) continue;

      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header.stamp = t;
      cloud.header.frame_id = frame_id_;
      cloud.height = 1;
      cloud.width  = static_cast<uint32_t>(N);
      cloud.is_dense = true;

      sensor_msgs::PointCloud2Modifier mod(cloud);
      mod.setPointCloud2FieldsByString(1, "xyz");
      mod.resize(cloud.width);

      sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");

      for (const auto& kv : points) {
        const auto& pt = kv.second;
        *it_x = static_cast<float>(pt.x);
        *it_y = static_cast<float>(pt.y);
        *it_z = static_cast<float>(pt.z);
        ++it_x; ++it_y; ++it_z;
      }

      pub->publish(cloud);
    }
  }

private:
  // Params
  std::string frame_id_, input_topic_, base_topic_;
  double rate_hz_;
  bool apply_orb_axes_;

  // Sub/timer
  rclcpp::Subscription<orbslam2_msgs::msg::MapPoint>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Data: agent -> (id -> point)
  std::unordered_map<std::string, PointsById> store_;
  // Dynamic pubs per agent
  std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pubs_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgentCloudFanout>());
  rclcpp::shutdown();
  return 0;
}
