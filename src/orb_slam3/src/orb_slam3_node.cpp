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
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <cmath>
#include <vector>
#include <array>
#include <algorithm>


class ORBSLAM3Node : public rclcpp::Node {
public:
    ORBSLAM3Node(const rclcpp::NodeOptions &options)
        : Node("orb_slam3_node", options)
        {
            RCLCPP_INFO(this->get_logger(), "Node initialized as '%s' (ORB-SLAM3)", this->get_name());

            std::string vocab_file = this->declare_parameter<std::string>("vocab_file");
            std::string settings_file = this->declare_parameter<std::string>("settings_file");
            output_dir_ = this->declare_parameter<std::string>("output_dir", ".");

            // Offline neural-SDF dataset saving: when enabled, every tracked
            // frame's RGB + raw depth is written to a slam_00N folder, and at
            // shutdown the optimized keyframe poses are exported so the folder
            // matches the neural-sdf-lab/rgbd_pipeline dataset format.
            save_keyframes_    = this->declare_parameter<bool>("save_keyframes", false);
            result_dir_param_  = this->declare_parameter<std::string>("result_dir", "");

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

            // fx_/expected_width_ are set above, so the dataset (and its
            // metadata.json intrinsics) can be created now.
            if (save_keyframes_) {
                setupKeyframeDataset();
                save_worker_ = std::thread(&ORBSLAM3Node::saveWorkerLoop, this);
            }
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
        cv::Mat depth_raw16;   // original 16UC1 depth in millimeters (for saving)
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

            if (save_keyframes_) depth_raw16 = cv_ptr->image.clone();
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

        // Stage the frame for the offline dataset. Only frames tracked OK can
        // become keyframes; non-keyframe frames are pruned at shutdown, so the
        // final slam_00N folder holds only keyframe RGB/depth + poses.
        if (save_keyframes_ && trackingOk &&
            !bgr_image.empty() && !depth_raw16.empty()) {
            enqueueFrameSave(bgr_image, depth_raw16, timestamp);
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

    // ================= Offline neural-SDF dataset saving =================
    // Produces a dataset directory in the exact layout consumed by
    // neural-sdf-lab/rgbd_pipeline (rgbd_dataset.py): metadata.json,
    // timestamps.csv, rgb/NNNNNN.png (RGB8), depth/NNNNNN.png (raw Z16, mm),
    // and poses.json (camera_to_world, meters). Only ORB-SLAM3 keyframes,
    // with globally-optimized poses, survive in the final folder.

    struct SavedFrame {
        int frame_id;
        double timestamp;
        std::string host_utc;
        std::string rgb_rel;
        std::string depth_rel;
    };

    struct PoseRecord {
        int frame_id;
        double timestamp;
        std::array<double, 16> matrix;  // row-major camera_to_world
    };

    struct SaveJob {
        std::string rgb_path;
        std::string depth_path;
        cv::Mat rgb_bgr;
        cv::Mat depth16;
    };

    static std::string utcNow() {
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm tm_utc{};
        gmtime_r(&t, &tm_utc);
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S+00:00", &tm_utc);
        return std::string(buf);
    }

    // Pick the next unused slam_00N directory under the result root.
    void setupKeyframeDataset() {
        namespace fs = std::filesystem;
        std::string root = result_dir_param_;
        if (root.empty()) {
            const char* home = std::getenv("HOME");
            root = std::string(home ? home : ".") + "/result";
        }
        fs::path result_root(root);
        std::error_code ec;
        fs::create_directories(result_root, ec);

        int next = 1;
        if (fs::exists(result_root)) {
            for (const auto& entry : fs::directory_iterator(result_root, ec)) {
                if (!entry.is_directory()) continue;
                const std::string name = entry.path().filename().string();
                if (name.rfind("slam_", 0) != 0) continue;
                try {
                    int n = std::stoi(name.substr(5));
                    if (n >= next) next = n + 1;
                } catch (const std::exception&) { /* ignore non-numeric */ }
            }
        }

        std::ostringstream folder;
        folder << "slam_" << std::setw(3) << std::setfill('0') << next;
        dataset_dir_ = result_root / folder.str();
        fs::create_directories(dataset_dir_ / "rgb", ec);
        fs::create_directories(dataset_dir_ / "depth", ec);

        dataset_created_utc_ = utcNow();
        writeMetadata(/*frame_count=*/0, /*finished=*/false);

        RCLCPP_INFO(this->get_logger(),
            "Keyframe saving ENABLED. Writing dataset to: %s",
            dataset_dir_.c_str());
    }

    void writeMetadata(int frame_count, bool finished) {
        std::ofstream f(dataset_dir_ / "metadata.json");
        f << std::setprecision(10);
        f << "{\n";
        f << "  \"schema_version\": 1,\n";
        f << "  \"created_utc\": \"" << dataset_created_utc_ << "\",\n";
        f << "  \"finished_utc\": " <<
            (finished ? "\"" + utcNow() + "\"" : std::string("null")) << ",\n";
        f << "  \"frame_count\": " << frame_count << ",\n";
        f << "  \"source\": \"orb_slam3_keyframes\",\n";
        f << "  \"agent\": \"" << agent_name_ << "\",\n";
        f << "  \"camera_backend\": \"ros2_realsense\",\n";
        f << "  \"width\": " << expected_width_ << ",\n";
        f << "  \"height\": " << expected_height_ << ",\n";
        f << "  \"color_format\": \"RGB8_PNG\",\n";
        f << "  \"depth_format\": \"Z16_PNG\",\n";
        f << "  \"depth_aligned_to\": \"color\",\n";
        // The node divides the incoming 16UC1 depth (millimeters) by 1000, so
        // one raw depth unit is one millimeter.
        f << "  \"depth_scale_m_per_unit\": 0.001,\n";
        f << "  \"intrinsics\": {\n";
        f << "    \"fx\": " << fx_ << ",\n";
        f << "    \"fy\": " << fy_ << ",\n";
        f << "    \"cx\": " << cx_ << ",\n";
        f << "    \"cy\": " << cy_ << ",\n";
        f << "    \"width\": " << expected_width_ << ",\n";
        f << "    \"height\": " << expected_height_ << "\n";
        f << "  }\n";
        f << "}\n";
    }

    // Called from the sync (tracking) thread. Records the frame and hands the
    // heavy PNG encoding to the writer thread so tracking stays real-time.
    void enqueueFrameSave(const cv::Mat& bgr, const cv::Mat& depth16, double ts) {
        char stem[16];
        std::snprintf(stem, sizeof(stem), "%06d", next_frame_id_);

        SavedFrame rec;
        rec.frame_id  = next_frame_id_;
        rec.timestamp = ts;
        rec.host_utc  = utcNow();
        rec.rgb_rel   = std::string("rgb/")   + stem + ".png";
        rec.depth_rel = std::string("depth/") + stem + ".png";
        saved_frames_.push_back(rec);
        ++next_frame_id_;

        SaveJob job;
        job.rgb_path   = (dataset_dir_ / rec.rgb_rel).string();
        job.depth_path = (dataset_dir_ / rec.depth_rel).string();
        job.rgb_bgr    = bgr.clone();
        job.depth16    = depth16.clone();
        {
            std::unique_lock<std::mutex> lk(save_queue_mtx_);
            // Backpressure: block briefly if the disk cannot keep up rather
            // than growing memory without bound.
            save_queue_cv_.wait(lk, [this] {
                return save_queue_.size() < kSaveQueueMax ||
                       save_worker_stop_.load();
            });
            save_queue_.push(std::move(job));
        }
        save_queue_cv_.notify_one();
    }

    void saveWorkerLoop() {
        const std::vector<int> png_params = {cv::IMWRITE_PNG_COMPRESSION, 1};
        while (true) {
            SaveJob job;
            {
                std::unique_lock<std::mutex> lk(save_queue_mtx_);
                save_queue_cv_.wait(lk, [this] {
                    return save_worker_stop_.load() || !save_queue_.empty();
                });
                if (save_queue_.empty()) break;  // empty + notified => stopping
                job = std::move(save_queue_.front());
                save_queue_.pop();
            }
            save_queue_cv_.notify_one();  // wake a producer waiting for space
            try {
                cv::imwrite(job.rgb_path, job.rgb_bgr, png_params);
                cv::imwrite(job.depth_path, job.depth16, png_params);
            } catch (const cv::Exception& e) {
                RCLCPP_ERROR(this->get_logger(),
                    "Failed to write dataset frame: %s", e.what());
            }
        }
    }

    // Runs at shutdown, after final bundle adjustment. Exports the optimized
    // keyframe trajectory, writes poses.json, prunes non-keyframe frames, and
    // rewrites the manifests so the folder is a keyframe-only rgbd dataset.
    void finalizeKeyframeDataset() {
        namespace fs = std::filesystem;

        // 1. Drain and stop the writer so every staged PNG is on disk.
        {
            std::lock_guard<std::mutex> lk(save_queue_mtx_);
            save_worker_stop_.store(true);
        }
        save_queue_cv_.notify_all();
        if (save_worker_.joinable()) save_worker_.join();

        // 2. Collect the globally-optimized poses of the MAIN map's keyframes.
        //    The active (current) map can be a small temporary map created
        //    after a tracking loss; the main map is the qualified, stable
        //    shared frame the agent broadcasts. Exporting the main map keeps
        //    poses.json in that shared frame and drops orphan temp-map frames.
        std::vector<ORB_SLAM3::KeyFrame*> vpKFs = slam_->GetMainMapKeyFrames();
        std::sort(vpKFs.begin(), vpKFs.end(),
                  [](ORB_SLAM3::KeyFrame* a, ORB_SLAM3::KeyFrame* b) {
                      return a->mnId < b->mnId;
                  });

        const std::string tum_path =
            (dataset_dir_ / "keyframe_trajectory_tum.txt").string();
        std::ofstream tum(tum_path);
        tum << std::fixed;

        std::vector<std::pair<double, std::array<double, 16>>> kf_poses;
        for (ORB_SLAM3::KeyFrame* pKF : vpKFs) {
            if (!pKF || pKF->isBad()) continue;
            const Sophus::SE3f Twc = pKF->GetPoseInverse();
            const Eigen::Matrix3f R = Twc.rotationMatrix();
            const Eigen::Vector3f t = Twc.translation();
            kf_poses.emplace_back(
                pKF->mTimeStamp,
                std::array<double, 16>{
                    R(0, 0), R(0, 1), R(0, 2), t(0),
                    R(1, 0), R(1, 1), R(1, 2), t(1),
                    R(2, 0), R(2, 1), R(2, 2), t(2),
                    0.0,     0.0,     0.0,     1.0});
            const Eigen::Quaternionf q = Twc.unit_quaternion();
            tum << std::setprecision(6) << pKF->mTimeStamp
                << std::setprecision(7)
                << ' ' << t(0) << ' ' << t(1) << ' ' << t(2)
                << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' '
                << q.w() << '\n';
        }
        if (kf_poses.empty()) {
            RCLCPP_WARN(this->get_logger(),
                "No MAIN map keyframes to export: no map reached the main-map "
                "quality gate this session, so poses.json will be empty.");
        }

        // 3. Match each keyframe timestamp to a staged frame_id. The keyframe
        //    stores the exact TrackRGBD timestamp we recorded, so matching is
        //    effectively exact; a small tolerance guards float round-trips.
        constexpr double kTol = 1e-3;
        std::vector<PoseRecord> records;
        std::vector<bool> keep(saved_frames_.size(), false);
        for (const auto& [ts, matrix] : kf_poses) {
            int best = -1;
            double best_dt = kTol;
            for (size_t i = 0; i < saved_frames_.size(); ++i) {
                double dt = std::fabs(saved_frames_[i].timestamp - ts);
                if (dt <= best_dt) { best_dt = dt; best = static_cast<int>(i); }
            }
            if (best >= 0) {
                keep[best] = true;
                records.push_back(
                    {saved_frames_[best].frame_id,
                     saved_frames_[best].timestamp, matrix});
            }
        }
        std::sort(records.begin(), records.end(),
                  [](const PoseRecord& a, const PoseRecord& b) {
                      return a.frame_id < b.frame_id;
                  });

        // 4. poses.json (schema mirrors rgbd_dataset.write_poses).
        writePosesJson(records);

        // 5. Prune every non-keyframe image and rewrite the manifests so the
        //    folder holds only keyframes.
        std::error_code ec;
        std::vector<SavedFrame> kept;
        for (size_t i = 0; i < saved_frames_.size(); ++i) {
            if (keep[i]) { kept.push_back(saved_frames_[i]); continue; }
            fs::remove(dataset_dir_ / saved_frames_[i].rgb_rel, ec);
            fs::remove(dataset_dir_ / saved_frames_[i].depth_rel, ec);
        }
        writeManifests(kept);
        writeMetadata(static_cast<int>(kept.size()), /*finished=*/true);

        RCLCPP_INFO(this->get_logger(),
            "Saved %zu keyframes (from %zu tracked frames) to %s",
            kept.size(), saved_frames_.size(), dataset_dir_.c_str());
    }

    void writePosesJson(const std::vector<PoseRecord>& records) {
        std::ofstream f(dataset_dir_ / "poses.json");
        f << std::setprecision(12);
        f << "{\n";
        f << "  \"schema_version\": 1,\n";
        f << "  \"convention\": \"camera_to_world\",\n";
        f << "  \"units\": \"meters\",\n";
        f << "  \"source\": \"orb_slam3_keyframes:" << agent_name_ << "\",\n";
        f << "  \"frames\": [\n";
        for (size_t r = 0; r < records.size(); ++r) {
            const PoseRecord& rec = records[r];
            f << "    {\n";
            f << "      \"frame_id\": " << rec.frame_id << ",\n";
            f << "      \"camera_timestamp_s\": " << rec.timestamp << ",\n";
            f << "      \"matrix\": [\n";
            for (int row = 0; row < 4; ++row) {
                f << "        [";
                for (int col = 0; col < 4; ++col) {
                    f << rec.matrix[row * 4 + col];
                    if (col < 3) f << ", ";
                }
                f << "]" << (row < 3 ? "," : "") << "\n";
            }
            f << "      ]\n";
            f << "    }" << (r + 1 < records.size() ? "," : "") << "\n";
        }
        f << "  ]\n";
        f << "}\n";
    }

    void writeManifests(const std::vector<SavedFrame>& frames) {
        std::ofstream csv(dataset_dir_ / "timestamps.csv");
        std::ofstream rgb(dataset_dir_ / "rgb.txt");
        std::ofstream depth(dataset_dir_ / "depth.txt");
        std::ofstream assoc(dataset_dir_ / "associations.txt");
        csv << "frame_id,camera_timestamp_s,host_timestamp_utc,rgb_file,depth_file\n";
        rgb << "# timestamp rgb_filename\n";
        depth << "# timestamp depth_filename\n";
        assoc << "# rgb_timestamp rgb_filename depth_timestamp depth_filename\n";
        csv << std::setprecision(6) << std::fixed;
        rgb << std::setprecision(6) << std::fixed;
        depth << std::setprecision(6) << std::fixed;
        assoc << std::setprecision(6) << std::fixed;
        for (const auto& fr : frames) {
            csv << fr.frame_id << ',' << fr.timestamp << ',' << fr.host_utc
                << ',' << fr.rgb_rel << ',' << fr.depth_rel << '\n';
            rgb << fr.timestamp << ' ' << fr.rgb_rel << '\n';
            depth << fr.timestamp << ' ' << fr.depth_rel << '\n';
            assoc << fr.timestamp << ' ' << fr.rgb_rel << ' '
                  << fr.timestamp << ' ' << fr.depth_rel << '\n';
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

            // Finalize the offline dataset using the now-optimized keyframe
            // poses (must run while slam_ is still alive for the Atlas export).
            if (save_keyframes_) {
                finalizeKeyframeDataset();
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

    // ---- Offline neural-SDF keyframe dataset saving ----
    bool save_keyframes_ = false;
    std::string result_dir_param_;
    std::filesystem::path dataset_dir_;
    std::string dataset_created_utc_;
    int next_frame_id_ = 0;

    std::vector<SavedFrame> saved_frames_;  // sync-thread only

    std::queue<SaveJob> save_queue_;
    std::mutex save_queue_mtx_;
    std::condition_variable save_queue_cv_;
    std::atomic<bool> save_worker_stop_{false};
    std::thread save_worker_;
    size_t kSaveQueueMax = 64;

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
