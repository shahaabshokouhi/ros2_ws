#include <cstring>
#include <array>

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

#include <orbslam2_msgs/msg/map_point.hpp>
#include <orbslam2_msgs/msg/map_point_array.hpp>

#include <MapPoint.h>
#include <System.h>

class MapReceiver : public rclcpp::Node {
public:
    MapReceiver()
    : Node("map_receiver")
    {
        // pub and sub
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/map_receiver/pointcloud", 10);

        map_sub_ = this->create_subscription<orbslam2_msgs::msg::MapPoint>(
            "/orb_slam2/single_mappoint", 10,
            std::bind(&MapReceiver::mapCallback, this, std::placeholders::_1)
        );
        
        // timer for publishing the cloud
        auto period = std::chrono::milliseconds(100); // 10 Hz
        timer_ = this->create_wall_timer(
            period, std::bind(&MapReceiver::publishCloud, this)
        );

        RCLCPP_INFO(this->get_logger(), "MapReceiver node initialized");
    }

private:
    void mapCallback(const orbslam2_msgs::msg::MapPoint::SharedPtr msg)
    {
        if (msg->is_bad) return;
        // Convert MapPoint message to cv::Mat and store
        cv::Mat point(1, 3, CV_32F);
        point.at<float>(0) = static_cast<float>(msg->position.z);
        point.at<float>(1) = static_cast<float>(-msg->position.x);
        point.at<float>(2) = static_cast<float>(-msg->position.y);
        point_cloud_.push_back(point);
    }

    void publishCloud()
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

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Subscription<orbslam2_msgs::msg::MapPoint>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<cv::Mat> point_cloud_;
};

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapReceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}