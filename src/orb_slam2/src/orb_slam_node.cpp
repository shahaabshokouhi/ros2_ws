#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <System.h>


class ImageViewerNode : public rclcpp::Node {
public:
    ImageViewerNode()
        : Node("orb_slam_node") 
        {
            RCLCPP_INFO(this->get_logger(), "Starting Image Viewer Node...");
            

            slam_ = std::make_unique<ORB_SLAM2::System>(
                "/home/jetson1/ORB_SLAM2/Vocabulary/ORBvoc.txt",
                "/home/jetson1/ORB_SLAM2/Examples/RGB-D/realsense.yaml",
                ORB_SLAM2::System::RGBD, false);

            cv::FileStorage fsettings(
                "/home/jetson1/ORB_SLAM2/Examples/RGB-D/realsense.yaml", 
                cv::FileStorage::READ
            );

            fx_ = fsettings["Camera.fx"];
            fy_ = fsettings["Camera.fy"];
            cx_ = fsettings["Camera.cx"];
            cy_ = fsettings["Camera.cy"];

            pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "orb_slam2/pointcloud", 10);
            pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "orb_slam2/pose", 10);
            path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
                "orb_slam2/path", 10);
            path_msg_.header.frame_id = "camera_color_optical_frame";

            color_sub_.subscribe(this, "/camera/realsense2_camera/color/image_raw");
            depth_sub_.subscribe(this, "/camera/realsense2_camera/depth/image_rect_raw");
                        
            sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), color_sub_, depth_sub_);
            sync_->registerCallback(std::bind(&ImageViewerNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

        }


private:

    void syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& color_msg,
                     const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
        
        cv::Mat depth_normalized;
        cv::Mat bgr_image;
        cv::Mat Tcw;

        // Process color image
        try {
            auto cv_ptr = cv_bridge::toCvShare(color_msg, sensor_msgs::image_encodings::RGB8);
            cv::cvtColor(cv_ptr->image, bgr_image, cv::COLOR_RGB2BGR);

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (color): %s", e.what());
        } catch (cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV exception (color): %s", e.what());
        }

        // Process depth image
        try {
            auto cv_ptr = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
            
            cv_ptr->image.convertTo(depth_normalized, CV_32FC1, 1.0 / 1000.0);
            depth_normalized.setTo(0.0f, cv_ptr->image == 0);

            // RCLCPP_INFO(this->get_logger(), "Sample depth value (meters): %f", depth_normalized.at<float>(240, 320));

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (depth): %s", e.what());
        }

        double timestamp = rclcpp::Time(color_msg->header.stamp).seconds();

        // Process images with ORB-SLAM2
        if (slam_) {
                Tcw = slam_->TrackRGBD(
                bgr_image,
                depth_normalized,
                timestamp
            );
        }
        
        if (Tcw.empty()) {
            RCLCPP_WARN(this->get_logger(), "Tracking failed");
        } else {
            RCLCPP_INFO(this->get_logger(), "Tracking succeeded");
        }

        path_msg_.header.stamp = color_msg->header.stamp;

        // Create PointCloud2 message
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = color_msg->header.stamp;
        cloud_msg.header.frame_id = "camera_color_optical_frame";
        cloud_msg.height = 1;

        std::vector<float> points;

        if (!Tcw.empty()) {
            cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
            cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
            
            cv::Mat Rwc = Rcw.t();
            cv::Mat twc = -Rwc * tcw;

            for (int v = 0; v < depth_normalized.rows; ++v) {
                for (int u = 0; u < depth_normalized.cols; ++u)
                {
                    float depth = depth_normalized.at<float>(v, u);
                    if (depth > 4 || depth < 0.2) continue;
                    float z = depth;
                    float x = (u - cx_) * z / fx_;
                    float y = (v - cy_) * z / fy_;

                    points.push_back(z);
                    points.push_back(-x);
                    points.push_back(-y);
                }
                
            }

            cloud_msg.width = points.size() / 3;
            cloud_msg.is_dense = false;

            sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
            modifier.setPointCloud2FieldsByString(1, "xyz");
            modifier.resize(cloud_msg.width);

            sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

            for (size_t i = 0; i < points.size(); i += 3) {
                *iter_x = points[i];
                *iter_y = points[i + 1];
                *iter_z = points[i + 2];
                ++iter_x; ++iter_y; ++iter_z;
            }

            pointcloud_pub_->publish(cloud_msg);
        }


        // Publish the pose
        if (!Tcw.empty()) {
            cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
            cv::Mat tcw = Tcw.rowRange(0, 3).col(3);

            cv::Mat Rwc = Rcw.t();
            cv::Mat twc = -Rwc * tcw;

            tf2::Matrix3x3 tf_rot(
                Rwc.at<float>(0, 0), Rwc.at<float>(0, 1), Rwc.at<float>(0, 2),
                Rwc.at<float>(1, 0), Rwc.at<float>(1, 1), Rwc.at<float>(1, 2),
                Rwc.at<float>(2, 0), Rwc.at<float>(2, 1), Rwc.at<float>(2, 2));
            
            tf2::Quaternion tf_quat;
            tf_rot.getRotation(tf_quat);

            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = color_msg->header.stamp;
            pose_msg.header.frame_id = "camera_frame";
            pose_msg.pose.position.x = twc.at<float>(2);
            pose_msg.pose.position.y = -twc.at<float>(0);
            pose_msg.pose.position.z = -twc.at<float>(1);
            pose_msg.pose.orientation.x = -tf_quat.z();
            pose_msg.pose.orientation.y = tf_quat.x();
            pose_msg.pose.orientation.z = -tf_quat.y();
            pose_msg.pose.orientation.w = tf_quat.w();

            path_msg_.poses.push_back(pose_msg);

            pose_pub_->publish(pose_msg);

        }

        // Publish the path
        path_pub_->publish(path_msg_);

    }

    void colorCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        RCLCPP_INFO(this->get_logger(), "Received color image: width=%d, height=%d, encoding=%s",
                    msg->width, msg->height, msg->encoding.c_str());

        try {
            // Try converting to bgr8
            auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
            cv::Mat bgr_image;
            cv::cvtColor(cv_ptr->image, bgr_image, cv::COLOR_RGB2BGR);

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (color): %s", e.what());
        } catch (cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV exception (color): %s", e.what());
        }
    }

    void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        RCLCPP_INFO(this->get_logger(), "Received depth image: width=%d, height=%d, encoding=%s",
                    msg->width, msg->height, msg->encoding.c_str());

        try {
            auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            
            cv::Mat depth_normalized;
            cv_ptr->image.convertTo(depth_normalized, CV_32FC1, 1.0 / 1000.0);
            // Handle invalid values (set 0 in input to 0.0 in float matrix)
            depth_normalized.setTo(0.0f, cv_ptr->image == 0);

            // Log a sample value to verify conversion
            RCLCPP_INFO(this->get_logger(), "Sample depth value (meters): %f", depth_normalized.at<float>(240, 320));

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (depth): %s", e.what());
        }
    }

    std::unique_ptr<ORB_SLAM2::System> slam_;
    nav_msgs::msg::Path path_msg_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

    message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    float fx_ = 0.0f;
    float fy_ = 0.0f;
    float cx_ = 0.0f;
    float cy_ = 0.0f;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageViewerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}