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
#include <KeyFrame.h>

#include <atomic>
#include <unordered_map>
#include <mutex>




class ORBSLAM2Node : public rclcpp::Node {
public:
    ORBSLAM2Node(const rclcpp::NodeOptions &options)
        : Node("orb_slam2_node", options) 
        {
            RCLCPP_INFO(this->get_logger(), "Node initialized as  '%s'", this->get_name());

            std::string vocab_file = this->declare_parameter<std::string>("vocab_file");
            std::string settings_file = this->declare_parameter<std::string>("settings_file");
            
            agent_name_ = this->get_name();
            const std::string color_topic = std::string("/") + agent_name_ + std::string("/camera/realsense2_camera/color/image_raw");
            const std::string depth_topic = std::string("/") + agent_name_ + std::string("/camera/realsense2_camera/depth/image_rect_raw");    

            slam_ = std::make_unique<ORB_SLAM2::System>(
                vocab_file,
                settings_file,
                ORB_SLAM2::System::RGBD, false, agent_name_);

            cv::FileStorage fsettings(
                settings_file,
                cv::FileStorage::READ
            );

            fx_ = fsettings["Camera.fx"];
            fy_ = fsettings["Camera.fy"];
            cx_ = fsettings["Camera.cx"];
            cy_ = fsettings["Camera.cy"];

            auto qos = rclcpp::QoS(rclcpp::KeepLast(5000)).reliable().durability_volatile();

            pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "orb_slam2/pointcloud", 10);
            pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "orb_slam2/pose", 10);
            mappoint_pub_ = this->create_publisher<orbslam2_msgs::msg::MapPointArray>(
                "orb_slam2/mappoints", 10);
            single_mappoint_pub_ = this->create_publisher<orbslam2_msgs::msg::MapPoint>(
                "orb_slam2/single_mappoint", qos);
            path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
                "orb_slam2/path", 10);
            path_msg_.header.frame_id = "camera_color_optical_frame";

            color_sub_.subscribe(this, color_topic);
            depth_sub_.subscribe(this, depth_topic);

            using std::placeholders::_1;
            mappoint_sub_ = this->create_subscription<orbslam2_msgs::msg::MapPoint>(
                "/orb_slam2/single_mappoint", qos,
                std::bind(&ORBSLAM2Node::importMapPointCallback, this, _1));
                        
            sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), color_sub_, depth_sub_);
            sync_->registerCallback(std::bind(&ORBSLAM2Node::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

            // debugging --------------------
            // statsTimer_ = this->create_wall_timer(
            //     std::chrono::seconds(1),
            //     [this]() {
            //         uint64_t pub = publishedCount_.load();
            //         uint64_t rec = receivedCount_.load();
            //         uint64_t imp = importedCount_.load();

            //         RCLCPP_INFO(this->get_logger(),
            //         "[MP STATS] published=%lu received=%lu imported=%lu (recv/pub=%.2f%%)",
            //         pub, rec, imp,
            //         (pub ? (100.0 * double(rec) / double(pub)) : 0.0)
            //         );

            //         // Optional per-agent dump (not every second if it’s too spammy)
            //         std::lock_guard<std::mutex> lk(perAgentMtx_);
            //         for (auto &kv : receivedByAgent_) {
            //         RCLCPP_INFO(this->get_logger(), "  [MP STATS] receivedByAgent[%s]=%lu",
            //                     kv.first.c_str(), kv.second);
            //         }
            //     }
            // );

        }


private:

    void syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& color_msg,
                     const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
        
        cv::Mat depth_normalized;
        cv::Mat bgr_image;
        cv::Mat Tcw;
        std::vector<ORB_SLAM2::MapPoint*> vpHighQualityMapPoints;
        std::vector<float> points;

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
            timestamp);

            vpHighQualityMapPoints = slam_->PopNewHighQualityMapPoints();
            // if (vpHighQualityMapPoints.size()){
            //     std::cout << "Found " << vpHighQualityMapPoints.size() << " new MapPoints to publish" << std::endl;
            //     for (const auto &kv : slam_->mpHQmanager->mImportedPointsByAgent)
            //         std::cout << kv.first << ": " << kv.second.size() << " points" << std::endl;
            // }

            // for (const auto &kv : slam_->mpHQmanager->mImportedPointsByAgent)
            //     std::cout << kv.first << ": " << kv.second.size() << " points" << std::endl;

        }
        
        if (Tcw.empty()) {
            RCLCPP_WARN(this->get_logger(), "Tracking failed");}
    

        // Create PointCloud2 message
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = color_msg->header.stamp;
        cloud_msg.header.frame_id = "camera_color_optical_frame";
        cloud_msg.height = 1;

        // Create MapPointArray message
        orbslam2_msgs::msg::MapPointArray mappoints_msg;
        mappoints_msg.header.stamp = color_msg->header.stamp;
        mappoints_msg.header.frame_id = "camera_color_optical_frame";
        mappoints_msg.agent_name = agent_name_;

        if (!Tcw.empty()) {
            cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
            cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
            
            cv::Mat Rwc = Rcw.t();
            cv::Mat twc = -Rwc * tcw;
            cv::Mat Twc = cv::Mat::eye(4, 4, CV_32F);
            Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));
            twc.copyTo(Twc.rowRange(0, 3).col(3));
            
            // pose and path message
            tf2::Matrix3x3 tf_rot(
                Rwc.at<float>(0, 0), Rwc.at<float>(0, 1), Rwc.at<float>(0, 2),
                Rwc.at<float>(1, 0), Rwc.at<float>(1, 1), Rwc.at<float>(1, 2),
                Rwc.at<float>(2, 0), Rwc.at<float>(2, 1), Rwc.at<float>(2, 2));
            
            tf2::Quaternion tf_quat;
            tf_rot.getRotation(tf_quat);

            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = color_msg->header.stamp;
            path_msg_.header.stamp = color_msg->header.stamp;
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

            if (publish_mappoints) {
                mappoints_msg.points.reserve(vpHighQualityMapPoints.size());
                for(auto* pMP : vpHighQualityMapPoints) {
                    if (!pMP || pMP->isBad()) continue;
                    mappoints_msg.points.push_back(toMsg(pMP));
                }
                mappoint_pub_->publish(mappoints_msg);
            }

            if (publish_single_mappoint && !vpHighQualityMapPoints.empty()) {
                std::cout << "\ndoing single mappoint pub..." << std::endl;
                // Publish the first valid map point as a single message
                for (auto* pMP : vpHighQualityMapPoints) {
                    if (!pMP) continue;
                    
                    single_mappoint_pub_->publish(toMsg(pMP));
                    publishedCount_++;
                    pMP->SentToOther(true);
                }
            }

            // point cloud message
            if (publish_cloud){

                for (ORB_SLAM2::MapPoint* pMP : vpHighQualityMapPoints){
                    if (!pMP || pMP->isBad()) continue;
                    cv::Mat pos = pMP->GetWorldPos();

                    points.push_back(pos.at<float>(2));
                    points.push_back(-pos.at<float>(0));
                    points.push_back(-pos.at<float>(1));
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
            
        }

        // Publish the path
        path_pub_->publish(path_msg_);

    }

    orbslam2_msgs::msg::MapPoint toMsg(ORB_SLAM2::MapPoint* pMP) {
        orbslam2_msgs::msg::MapPoint m;
        // ID
        m.agent_name = agent_name_;
        m.id = static_cast<uint64_t>(pMP->mnId);

        // position
        const cv::Mat X = pMP->GetWorldPos();
        m.position.x = static_cast<double>(X.at<float>(0));
        m.position.y = static_cast<double>(X.at<float>(1));
        m.position.z = static_cast<double>(X.at<float>(2));

        // noraml and distance invariance range
        const cv::Mat n = pMP->GetNormal();
        m.normal.x = static_cast<double>(n.at<float>(0));
        m.normal.y = static_cast<double>(n.at<float>(1));
        m.normal.z = static_cast<double>(n.at<float>(2));
        m.min_distance = pMP->GetMinDistanceInvariance();
        m.max_distance = pMP->GetMaxDistanceInvariance();

        // orb descriptors (1X32 CV_8U)
        const cv::Mat d = pMP->GetDescriptor();
        if (d.cols != 32 || d.type() != CV_8U) {
            RCLCPP_ERROR(this->get_logger(), "Invalid descriptor size or type");
        } else {
            std::memcpy(m.descriptor.data(), d.ptr<uint8_t>(), 32);
        }
        m.is_bad = pMP->isBad();
        m.is_high_quality = pMP->mbHighQaulity;
        m.stamp = this->get_clock()->now();

        const std::map<ORB_SLAM2::KeyFrame*, size_t> obs = pMP->GetObservations();
        m.keyframe_ids.reserve(obs.size());
        for (std::map<ORB_SLAM2::KeyFrame*, size_t>::const_iterator it = obs.begin();
            it != obs.end(); ++it) 
        {
            ORB_SLAM2::KeyFrame* pKF = it->first;
            if (!pKF || pKF->isBad()) continue;
            m.keyframe_ids.push_back(static_cast<uint64_t>(pKF->mnId));
        }

        return m;
    }

    void importMapPointCallback(const orbslam2_msgs::msg::MapPoint::SharedPtr msg)
    {

        // debugging ---
        receivedCount_++;
        {
        std::lock_guard<std::mutex> lk(perAgentMtx_);
        receivedByAgent_[msg->agent_name] += 1;
        }
        // ------------------------

        cv::Mat pos = (cv::Mat_<float>(3,1) <<
            msg->position.x,
            msg->position.y,
            msg->position.z);

        cv::Mat n = (cv::Mat_<float>(3,1) <<
            msg->normal.x,
            msg->normal.y,
            msg->normal.z);

        cv::Mat desc(1, 32, CV_8U);
        std::memcpy(desc.ptr<uint8_t>(), msg->descriptor.data(), 32);

        std::vector<int> keyframe_ids(msg->keyframe_ids.begin(), msg->keyframe_ids.end());

        auto* pMP = new ORB_SLAM2::MapPoint(
            msg->id,
            pos,
            n,
            desc,
            msg->min_distance,
            msg->max_distance,
            msg->is_bad,
            keyframe_ids);

        auto &agentVec = mImportedPointsByAgent[msg->agent_name];
        agentVec.push_back(pMP);

        if (agentVec.size() >= kBatchSize) {
            slam_->mpHQmanager->ImportHighQualityMapPoints(msg->agent_name, agentVec);
            importedCount_ += agentVec.size();  // should be 50 here
            agentVec.clear();
        }
    }

    
    // debuggin the mappoint sender issue------------------------
    std::atomic<uint64_t> publishedCount_{0};
    std::atomic<uint64_t> receivedCount_{0};
    std::atomic<uint64_t> importedCount_{0};

    // Per-agent received counters
    std::mutex perAgentMtx_;
    std::unordered_map<std::string, uint64_t> receivedByAgent_;

    // Log timer
    rclcpp::TimerBase::SharedPtr statsTimer_;

    //---------------------------------------------------------

    std::unique_ptr<ORB_SLAM2::System> slam_;
    nav_msgs::msg::Path path_msg_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<orbslam2_msgs::msg::MapPointArray>::SharedPtr mappoint_pub_;
    rclcpp::Publisher<orbslam2_msgs::msg::MapPoint>::SharedPtr single_mappoint_pub_;

    message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    rclcpp::Subscription<orbslam2_msgs::msg::MapPoint>::SharedPtr mappoint_sub_;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    float fx_ = 0.0f;
    float fy_ = 0.0f;
    float cx_ = 0.0f;
    float cy_ = 0.0f;
    bool publish_cloud = false;
    bool publish_mappoints = false;
    bool publish_single_mappoint = true;
    std::string agent_name_;
    std::map<std::string, std::vector<ORB_SLAM2::MapPoint*>> mImportedPointsByAgent;
    size_t kBatchSize = 50;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ORBSLAM2Node>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}