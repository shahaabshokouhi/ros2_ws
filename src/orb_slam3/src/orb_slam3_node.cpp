// ORB-SLAM3 multi-agent ROS 2 node — counterpart of orb_slam2/orb_slam_node.cpp.
//
// Wire compatibility: reuses orbslam2_msgs and the SAME inter-agent topics
// (/orb_slam2/single_mappoint, /orb_slam2/mappoints), so ORB-SLAM2 and
// ORB-SLAM3 agents can exchange map points in a mixed fleet. Per-agent
// visualization topics are published under the orb_slam3 prefix.
//
// Main-map policy (implemented in the ORB_SLAM3 library): only points from
// the agent's qualified MAIN map are popped for sharing; while the robot is
// lost in a temporary map, nothing new is shared until it merges into main.

#include <cstring>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
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
#include <Tracking.h>

#include <atomic>
#include <unordered_map>
#include <mutex>
#include <queue>
#include <thread>
#include <condition_variable>


class ORBSLAM3Node : public rclcpp::Node {
public:
    ORBSLAM3Node(const rclcpp::NodeOptions &options)
        : Node("orb_slam3_node", options)
        {
            RCLCPP_INFO(this->get_logger(), "Node initialized as '%s' (ORB-SLAM3)", this->get_name());

            std::string vocab_file = this->declare_parameter<std::string>("vocab_file");
            std::string settings_file = this->declare_parameter<std::string>("settings_file");
            output_dir_ = this->declare_parameter<std::string>("output_dir", ".");

            agent_name_ = this->get_name();
            const std::string color_topic = std::string("/") + agent_name_ + std::string("/camera/realsense2_camera/color/image_raw");
            const std::string depth_topic = std::string("/") + agent_name_ + std::string("/camera/realsense2_camera/depth/image_rect_raw");

            slam_ = std::make_unique<ORB_SLAM3::System>(
                vocab_file,
                settings_file,
                ORB_SLAM3::System::RGBD,
                /*bUseViewer=*/false,
                /*initFr=*/0,
                /*strSequence=*/std::string(),
                agent_name_);

            // ORB-SLAM3 settings use the Camera1.* keys; fall back to the
            // ORB-SLAM2 Camera.* names so either file format works here.
            cv::FileStorage fsettings(settings_file, cv::FileStorage::READ);
            fx_ = readIntrinsic(fsettings, "Camera1.fx", "Camera.fx");
            fy_ = readIntrinsic(fsettings, "Camera1.fy", "Camera.fy");
            cx_ = readIntrinsic(fsettings, "Camera1.cx", "Camera.cx");
            cy_ = readIntrinsic(fsettings, "Camera1.cy", "Camera.cy");
            expected_width_  = (int)fsettings["Camera.width"];
            expected_height_ = (int)fsettings["Camera.height"];

            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            auto qos = rclcpp::QoS(rclcpp::KeepLast(5000)).reliable().durability_volatile();

            hq_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                agent_name_ + "/orb_slam3/hq_pointcloud", 10);
            merged_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                agent_name_ + "/orb_slam3/merged_map", 10);
            pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                agent_name_ + "/orb_slam3/pose", 10);
            // Inter-agent topics: identical to the ORB-SLAM2 node for wire
            // compatibility in a mixed fleet — do not rename.
            mappoint_pub_ = this->create_publisher<orbslam2_msgs::msg::MapPointArray>(
                "orb_slam2/mappoints", 10);
            single_mappoint_pub_ = this->create_publisher<orbslam2_msgs::msg::MapPoint>(
                "orb_slam2/single_mappoint", qos);
            path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
                agent_name_ + "/orb_slam3/path", 10);
            image_plane_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                agent_name_ + "/orb_slam3/image_plane", 10);
            path_msg_.header.frame_id = "map";

            color_sub_.subscribe(this, color_topic);
            depth_sub_.subscribe(this, depth_topic);

            using std::placeholders::_1;
            mappoint_sub_ = this->create_subscription<orbslam2_msgs::msg::MapPoint>(
                "/orb_slam2/single_mappoint", qos,
                std::bind(&ORBSLAM3Node::importMapPointCallback, this, _1));

            // Background worker thread: processes ImportHighQualityMapPoints
            // batches so the ROS callback thread is never blocked.
            import_worker_ = std::thread(&ORBSLAM3Node::importWorkerLoop, this);

            merged_map_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(2000),
                std::bind(&ORBSLAM3Node::publishMergedMap, this));

            sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), color_sub_, depth_sub_);
            sync_->registerCallback(std::bind(&ORBSLAM3Node::syncCallback, this, std::placeholders::_1, std::placeholders::_2));
        }


private:

    static float readIntrinsic(const cv::FileStorage& fs,
                               const char* primary, const char* fallback) {
        cv::FileNode n = fs[primary];
        if (!n.empty()) return (float)n;
        return (float)fs[fallback];
    }

    void syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& color_msg,
                     const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {

        cv::Mat depth_normalized;
        cv::Mat bgr_image;
        std::vector<ORB_SLAM3::MapPoint*> vpHighQualityMapPoints;

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
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (depth): %s", e.what());
        }

        double timestamp = rclcpp::Time(color_msg->header.stamp).seconds();

        // Process images with ORB-SLAM3
        Sophus::SE3f Tcw;
        bool trackingOk = false;
        if (slam_) {

            if (bgr_image.empty() || depth_normalized.empty()) {
                RCLCPP_WARN_ONCE(this->get_logger(), "Skipping frame: empty color or depth image");
                return;
            }

            // The camera MUST stream the resolution the calibration was made
            // for. Newer realsense2_camera drivers ignore the old
            // color_width/height params and silently stream 1280x720, which
            // corrupts all SLAM geometry (wrong fx/cx for every unprojection).
            if (expected_width_ > 0 &&
                (bgr_image.cols != expected_width_ || bgr_image.rows != expected_height_)) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Image resolution %dx%d does NOT match settings %dx%d — SLAM geometry "
                    "will be wrong! Fix the camera profile (rgb_camera.color_profile) "
                    "in the launch file. Dropping frames.",
                    bgr_image.cols, bgr_image.rows, expected_width_, expected_height_);
                return;
            }
            if (depth_normalized.cols != bgr_image.cols || depth_normalized.rows != bgr_image.rows) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Depth resolution %dx%d does not match color %dx%d — is the aligned "
                    "depth topic being remapped correctly? Dropping frames.",
                    depth_normalized.cols, depth_normalized.rows, bgr_image.cols, bgr_image.rows);
                return;
            }

            try {
                Tcw = slam_->TrackRGBD(bgr_image, depth_normalized, timestamp);
                trackingOk = (slam_->GetTrackingState() == ORB_SLAM3::Tracking::OK);
                vpHighQualityMapPoints = slam_->PopNewHighQualityMapPoints();
            } catch (const cv::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "[TrackRGBD] cv::Exception: %s", e.what());
                return;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "[TrackRGBD] exception: %s", e.what());
                return;
            }
        }

        if (!trackingOk && !trackingFailed) {
            RCLCPP_WARN(this->get_logger(), "Tracking failed");
            trackingFailed = true;
        }

        // Create MapPointArray message
        orbslam2_msgs::msg::MapPointArray mappoints_msg;
        mappoints_msg.header.stamp = color_msg->header.stamp;
        mappoints_msg.header.frame_id = "camera_color_optical_frame";
        mappoints_msg.agent_name = agent_name_;

        if (trackingOk) {
            trackingFailed = false;

            const Sophus::SE3f Twc = Tcw.inverse();
            const Eigen::Matrix3f Rwc = Twc.rotationMatrix();
            const Eigen::Vector3f twc = Twc.translation();

            // pose and path message
            tf2::Matrix3x3 tf_rot(
                Rwc(0, 0), Rwc(0, 1), Rwc(0, 2),
                Rwc(1, 0), Rwc(1, 1), Rwc(1, 2),
                Rwc(2, 0), Rwc(2, 1), Rwc(2, 2));

            // Convert from ORB-SLAM world frame (Z-forward, X-right, Y-down)
            // to ROS map frame (X-forward, Y-left, Z-up).
            // Axis mapping: x_ros = z_orb, y_ros = -x_orb, z_ros = -y_orb
            tf2::Matrix3x3 R_orb_to_ros(
                 0,  0,  1,
                -1,  0,  0,
                 0, -1,  0);
            tf2::Matrix3x3 R_map_cam = R_orb_to_ros * tf_rot;
            tf2::Quaternion q_map_cam;
            R_map_cam.getRotation(q_map_cam);

            const float tx =  twc(2);
            const float ty = -twc(0);
            const float tz = -twc(1);

            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = color_msg->header.stamp;
            path_msg_.header.stamp = color_msg->header.stamp;
            pose_msg.header.frame_id = "map";
            pose_msg.pose.position.x = tx;
            pose_msg.pose.position.y = ty;
            pose_msg.pose.position.z = tz;
            pose_msg.pose.orientation.x = q_map_cam.x();
            pose_msg.pose.orientation.y = q_map_cam.y();
            pose_msg.pose.orientation.z = q_map_cam.z();
            pose_msg.pose.orientation.w = q_map_cam.w();

            path_msg_.poses.push_back(pose_msg);
            pose_pub_->publish(pose_msg);

            geometry_msgs::msg::TransformStamped tf_stamped;
            tf_stamped.header.stamp = color_msg->header.stamp;
            tf_stamped.header.frame_id = "map";
            tf_stamped.child_frame_id = "camera_color_optical_frame";
            tf_stamped.transform.translation.x = tx;
            tf_stamped.transform.translation.y = ty;
            tf_stamped.transform.translation.z = tz;
            tf_stamped.transform.rotation.x = q_map_cam.x();
            tf_stamped.transform.rotation.y = q_map_cam.y();
            tf_stamped.transform.rotation.z = q_map_cam.z();
            tf_stamped.transform.rotation.w = q_map_cam.w();
            tf_broadcaster_->sendTransform(tf_stamped);

            // Project image pixels to 3D in camera frame → floating image plane in RViz
            if (!bgr_image.empty() && image_plane_pub_->get_subscription_count() > 0) {
                constexpr int STEP = 8;          // sample every Nth pixel
                constexpr float PLANE_DEPTH = 0.5f; // meters in front of camera
                const int rows = bgr_image.rows;
                const int cols = bgr_image.cols;

                sensor_msgs::msg::PointCloud2 plane_msg;
                plane_msg.header.stamp = color_msg->header.stamp;
                plane_msg.header.frame_id = "camera_color_optical_frame";
                plane_msg.height = 1;
                plane_msg.is_dense = true;

                sensor_msgs::PointCloud2Modifier mod(plane_msg);
                mod.setPointCloud2FieldsByString(2, "xyz", "rgb");
                mod.resize(((rows + STEP - 1) / STEP) * ((cols + STEP - 1) / STEP));

                sensor_msgs::PointCloud2Iterator<float>   it_x(plane_msg, "x");
                sensor_msgs::PointCloud2Iterator<float>   it_y(plane_msg, "y");
                sensor_msgs::PointCloud2Iterator<float>   it_z(plane_msg, "z");
                sensor_msgs::PointCloud2Iterator<float>   it_rgb(plane_msg, "rgb");

                size_t count = 0;
                for (int v = 0; v < rows; v += STEP) {
                    for (int u = 0; u < cols; u += STEP) {
                        *it_x = (u - cx_) / fx_ * PLANE_DEPTH;
                        *it_y = (v - cy_) / fy_ * PLANE_DEPTH;
                        *it_z = PLANE_DEPTH;
                        const cv::Vec3b& px = bgr_image.at<cv::Vec3b>(v, u);
                        uint32_t rgb = ((uint32_t)px[2] << 16) | ((uint32_t)px[1] << 8) | (uint32_t)px[0];
                        std::memcpy(&(*it_rgb), &rgb, sizeof(float));
                        ++it_x; ++it_y; ++it_z; ++it_rgb;
                        ++count;
                    }
                }
                plane_msg.width = static_cast<uint32_t>(count);
                mod.resize(count);
                image_plane_pub_->publish(plane_msg);
            }

            if (publish_mappoints) {
                mappoints_msg.points.reserve(vpHighQualityMapPoints.size());
                for(auto* pMP : vpHighQualityMapPoints) {
                    if (!pMP || pMP->isBad()) continue;
                    mappoints_msg.points.push_back(toMsg(pMP));
                }
                mappoint_pub_->publish(mappoints_msg);
            }

            if (publish_single_mappoint && !vpHighQualityMapPoints.empty()) {
                for (auto* pMP : vpHighQualityMapPoints) {
                    if (!pMP) continue;

                    single_mappoint_pub_->publish(toMsg(pMP));
                    publishedCount_++;
                    pMP->SentToOther(true);
                }
            }

        }

        // Publish the path
        path_pub_->publish(path_msg_);
    }

    orbslam2_msgs::msg::MapPoint toMsg(ORB_SLAM3::MapPoint* pMP) {
        orbslam2_msgs::msg::MapPoint m;
        // ID
        m.agent_name = agent_name_;
        m.id = static_cast<uint64_t>(pMP->mnId);

        // position (main-map frame — the agent's stable shared frame)
        const Eigen::Vector3f X = pMP->GetWorldPos();
        m.position.x = static_cast<double>(X(0));
        m.position.y = static_cast<double>(X(1));
        m.position.z = static_cast<double>(X(2));

        // normal and distance invariance range
        const Eigen::Vector3f n = pMP->GetNormal();
        m.normal.x = static_cast<double>(n(0));
        m.normal.y = static_cast<double>(n(1));
        m.normal.z = static_cast<double>(n(2));
        m.min_distance = pMP->GetMinDistanceInvariance();
        m.max_distance = pMP->GetMaxDistanceInvariance();

        // orb descriptors (1x32 CV_8U)
        const cv::Mat d = pMP->GetDescriptor();
        if (d.cols != 32 || d.type() != CV_8U) {
            RCLCPP_ERROR(this->get_logger(), "Invalid descriptor size or type");
        } else {
            std::memcpy(m.descriptor.data(), d.ptr<uint8_t>(), 32);
        }
        m.is_bad = pMP->isBad();
        m.is_high_quality = pMP->mbHighQaulity;
        m.stamp = this->get_clock()->now();

        const std::map<ORB_SLAM3::KeyFrame*, std::tuple<int,int>> obs = pMP->GetObservations();
        m.keyframe_ids.reserve(obs.size());
        for (auto it = obs.begin(); it != obs.end(); ++it)
        {
            ORB_SLAM3::KeyFrame* pKF = it->first;
            if (!pKF || pKF->isBad()) continue;
            m.keyframe_ids.push_back(static_cast<uint64_t>(pKF->mnId));
        }

        return m;
    }

    void publishMergedMap() {
        if (!slam_ || !slam_->mpHQmanager) return;

        std::vector<cv::Point3f> pts = slam_->mpHQmanager->ExportMergedMap();
        if (pts.empty()) return;

        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = this->get_clock()->now();
        cloud_msg.header.frame_id = "map";
        cloud_msg.height = 1;
        cloud_msg.is_dense = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(pts.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

        for (const auto& p : pts) {
            *iter_x =  p.z; *iter_y = -p.x; *iter_z = -p.y;
            ++iter_x; ++iter_y; ++iter_z;
        }

        cloud_msg.width = static_cast<uint32_t>(pts.size());
        merged_map_pub_->publish(cloud_msg);
    }

    void importMapPointCallback(const orbslam2_msgs::msg::MapPoint::SharedPtr msg)
    {
        // Ignore our own points echoed back on the shared topic
        if (msg->agent_name == agent_name_) return;

        receivedCount_++;

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

        auto* pMP = new ORB_SLAM3::MapPoint(
            msg->id,
            pos,
            n,
            desc,
            msg->min_distance,
            msg->max_distance,
            msg->is_bad,
            keyframe_ids);
        pMP->mbHighQaulity = msg->is_high_quality;

        auto &agentVec = mImportedPointsByAgent[msg->agent_name];
        agentVec.push_back(pMP);

        if (agentVec.size() >= kBatchSize) {
            {
                std::lock_guard<std::mutex> lk(import_queue_mtx_);
                import_queue_.push({msg->agent_name, std::move(agentVec)});
            }
            import_queue_cv_.notify_one();
        }
    }

    void importWorkerLoop() {
        while (true) {
            ImportBatch batch;
            {
                std::unique_lock<std::mutex> lk(import_queue_mtx_);
                import_queue_cv_.wait(lk, [this] {
                    return import_worker_stop_.load() || !import_queue_.empty();
                });
                if (import_worker_stop_.load() && import_queue_.empty()) break;
                batch = std::move(import_queue_.front());
                import_queue_.pop();
            }
            if (slam_ && slam_->mpHQmanager) {
                try {
                    slam_->mpHQmanager->ImportHighQualityMapPoints(batch.agent_name, batch.points);
                    importedCount_ += batch.points.size();
                } catch (const cv::Exception& e) {
                    std::cerr << "[importWorker] cv::Exception in ImportHighQualityMapPoints: "
                              << e.what() << "\n";
                } catch (const std::exception& e) {
                    std::cerr << "[importWorker] exception: " << e.what() << "\n";
                }
            }
        }
    }

    std::atomic<uint64_t> publishedCount_{0};
    std::atomic<uint64_t> receivedCount_{0};
    std::atomic<uint64_t> importedCount_{0};

    std::unique_ptr<ORB_SLAM3::System> slam_;
    nav_msgs::msg::Path path_msg_;

public:
    void onShutdown() {
        // Stop the background import worker before touching SLAM internals
        {
            std::lock_guard<std::mutex> lk(import_queue_mtx_);
            import_worker_stop_.store(true);
        }
        import_queue_cv_.notify_one();
        if (import_worker_.joinable()) import_worker_.join();

        if (slam_) {
            std::cout << "[ORBSLAM3Node] Shutting down SLAM (waiting for final BA)...\n";
            slam_->Shutdown();
            std::cout << "[ORBSLAM3Node] SLAM shutdown complete.\n";

            if (slam_->mpHQmanager) {
                std::string dir = output_dir_;
                if (!dir.empty() && dir.back() != '/') dir += '/';
                const std::string csv_path = dir + "mappoint_descriptors.csv";
                slam_->mpHQmanager->ExportMapPointDescriptorsCSV(csv_path);
            }
        }
    }

private:

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr image_plane_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr hq_pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_map_pub_;
    rclcpp::TimerBase::SharedPtr merged_map_timer_;
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
    int expected_width_ = 0;
    int expected_height_ = 0;
    bool publish_mappoints = false;
    bool publish_single_mappoint = true;
    bool trackingFailed = false;
    std::string agent_name_;
    std::string output_dir_;
    std::map<std::string, std::vector<ORB_SLAM3::MapPoint*>> mImportedPointsByAgent;
    size_t kBatchSize = 50;

    // Background worker thread for ImportHighQualityMapPoints
    struct ImportBatch {
        std::string agent_name;
        std::vector<ORB_SLAM3::MapPoint*> points;
    };
    std::queue<ImportBatch> import_queue_;
    std::mutex import_queue_mtx_;
    std::condition_variable import_queue_cv_;
    std::atomic<bool> import_worker_stop_{false};
    std::thread import_worker_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ORBSLAM3Node>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    node->onShutdown();
    rclcpp::shutdown();
    return 0;
}
