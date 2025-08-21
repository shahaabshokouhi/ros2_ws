#include <cstring>
#include <array>
#include <string>
#include <memory>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <orbslam2_msgs/msg/map_point.hpp>

#include <MapPoint.h>
#include <System.h>

class MapReceiver : public rclcpp::Node {
public:
    MapReceiver()
    : Node("map_receiver")
    {   
        // Node parameters
        frame_id_ = declare_parameter<std::string>("output_frame", "map");
        apply_transformation_ = declare_parameter<bool>("apply_transformation", true);
        publish_rate_ = declare_parameter<double>("publish_rate", 5.0);
        publish_mode_ = declare_parameter<std::string>("publish_mode", "per_agent");
        base_output_topic_ = declare_parameter<std::string>("base_output_topic", "/map_receiver");
        auto period = std::chrono::milliseconds(static_cast<int>(
            1000.0 / std::max(0.1, publish_rate_))); // 5 Hz

        // pub and sub
        if (publish_mode_ == "all") {

            map_sub_ = this->create_subscription<orbslam2_msgs::msg::MapPoint>(
            "/orb_slam2/single_mappoint", rclcpp::QoS(200).best_effort(),
            std::bind(&MapReceiver::allCallback, this, std::placeholders::_1));

            cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/map_receiver/pointcloud_raw", 10);

            // timer for publishing the cloud
            timer_ = this->create_wall_timer(
                period, std::bind(&MapReceiver::publishAll, this));

        } else if (publish_mode_ == "per_agent") {

            map_sub_ = this->create_subscription<orbslam2_msgs::msg::MapPoint>(
                "/orb_slam2/single_mappoint", rclcpp::QoS(200).best_effort(),
                std::bind(&MapReceiver::perAgentCallback, this, std::placeholders::_1));

            timer_ = this->create_wall_timer(
                period, std::bind(&MapReceiver::publishPerAgent, this));

        }

        RCLCPP_INFO(this->get_logger(), "MapReceiver node initialized");
    }

private:
    using PointsById = std::unordered_map<uint64_t, geometry_msgs::msg::Point>;

    // Create or get a publisher for an agent
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& getPub(const std::string& agent) 
    {    
        auto it = pubs_.find(agent);
        if (it != pubs_.end()) return it->second;

        // createing a new publisher on topic /map_receiver/<agent>/pointcloud
        const std::string topic = base_output_topic_ + "/" + agent + "/pointcloud";
        auto pub = create_publisher<sensor_msgs::msg::PointCloud2>(
            topic, rclcpp::SensorDataQoS());
        RCLCPP_INFO(this->get_logger(), "Created publisher: %s", topic.c_str());
        return pubs_.emplace(agent, pub).first->second;
    }

    void perAgentCallback(const orbslam2_msgs::msg::MapPoint::SharedPtr msg) 
    {
        if(!msg) return;
        if (msg->is_bad) return;

        // Insert if new (per-agent de-dup by id)
        auto& per_agent = store_[msg->agent_name];

        geometry_msgs::msg::Point p = msg->position;
        if (apply_transformation_) {
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

    void publishPerAgent()
    {
        const rclcpp::Time t = this->now();
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
            cloud.width = static_cast<uint32_t>(N);
            cloud.is_dense = true;

            sensor_msgs::PointCloud2Modifier mod(cloud);
            mod.setPointCloud2FieldsByString(1, "xyz");
            mod.resize(cloud.width);

            sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

            for (const auto& point_pair : points) {
                const geometry_msgs::msg::Point& pt = point_pair.second;
                *iter_x = static_cast<float>(pt.x);
                *iter_y = static_cast<float>(pt.y);
                *iter_z = static_cast<float>(pt.z);
                ++iter_x; ++iter_y; ++iter_z;
            }
            pub->publish(cloud);
        }
    }

    void allCallback(const orbslam2_msgs::msg::MapPoint::SharedPtr msg)
    {
        if (!msg) return;
        if (msg->is_bad) return;

        // Convert MapPoint message to cv::Mat and store
        cv::Mat point(1, 3, CV_32F);
        point.at<float>(0) = static_cast<float>(msg->position.z);
        point.at<float>(1) = static_cast<float>(-msg->position.x);
        point.at<float>(2) = static_cast<float>(-msg->position.y);
        point_cloud_.push_back(point);
    }

    void publishAll()
    {
        if (point_cloud_.empty()) {
            return;
        }
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = this->get_clock()->now();
        cloud_msg.header.frame_id = "map";
        cloud_msg.height = 1;
        cloud_msg.width = point_cloud_.size();

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(point_cloud_.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

        for (const auto& pt : point_cloud_) {
            *iter_x = pt.at<float>(0);
            *iter_y = pt.at<float>(1);
            *iter_z = pt.at<float>(2);
            ++iter_x; ++iter_y; ++iter_z;
        }

        cloud_pub_->publish(cloud_msg);
        RCLCPP_INFO(this->get_logger(), "Published point cloud with %zu points", point_cloud_.size());
    }

    // Parameters
    std::string frame_id_;
    bool apply_transformation_;
    double publish_rate_;
    std::string publish_mode_;
    std::string base_output_topic_;

    // Subscribers and publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Subscription<orbslam2_msgs::msg::MapPoint>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // dynamic publishers
    std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pubs_;

    // Point Cloud storage
    std::vector<cv::Mat> point_cloud_;
    std::unordered_map<std::string, PointsById> store_;
};

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapReceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}