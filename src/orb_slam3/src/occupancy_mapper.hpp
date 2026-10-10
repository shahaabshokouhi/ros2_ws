// Occupancy grid for Nav2, built from the RGB-D depth images and anchored to
// ORB-SLAM3 keyframes.
//
// 1. Floor calibration. The camera's height, pitch and roll are unknown, so
//    the floor plane is fitted (RANSAC) in the first depth images. It defines
//    a level "base" frame on the floor under the camera: x forward, z up.
// 2. Virtual scans. Every time the robot has moved a little, the depth image
//    is cut to a height band above the floor (obstacles between
//    grid.obstacle_min_height and grid.obstacle_max_height, i.e. what the
//    robot would hit) and reduced to a fan of 1-degree bins up to
//    grid.max_range: the nearest obstacle per bin, or how far the floor is
//    seen to be free. The scan is stored RELATIVE TO ITS REFERENCE KEYFRAME.
// 3. Map. Once per grid.period every scan is drawn from its keyframe's
//    CURRENT pose (log-odds: free along each ray, occupied at a hit), so
//    local BA, loop closures and map merges move the obstacles with the map
//    instead of leaving doubled walls behind. Published as
//    nav_msgs/OccupancyGrid on /<agent>/map.
//
// Frames (names are parameters):
//   camera_floor   level frame on the floor under the camera (the scans' origin)
//   base_footprint level frame on the floor under the REAR AXLE, the point a
//                  car turns about: grid.camera_forward / grid.camera_left
//                  behind the camera (JetRacer: 0.21 m, 0)
//   map_nav        base_footprint at the start, a static child of ORB-SLAM3's
//                  "map" frame; the global frame for Nav2
// Two layouts, because a TF frame has exactly one parent:
//   default:    map -> camera_color_optical_frame (the node, SLAM pose)
//                 -> base_footprint, camera_floor (static, from the floor fit)
//   nav_frames: map -> map_nav -> odom -> base_footprint -> camera_*  (REP 105)
//               and the node stops publishing map -> camera. Same poses,
//               different tree. Who provides the robot's pose
//               (grid.pose_source):
//                 slam   (default) odom -> base_footprint IS the SLAM pose,
//                        published by this class every tracked frame;
//                        map_nav -> odom is the identity. Nav2's controller
//                        then steers on SLAM positions. The base driver must
//                        not publish odom -> base_footprint.
//                 wheels odom -> base_footprint is the base driver's wheel
//                        odometry; this class publishes the SLAM correction
//                        map_nav -> odom. Smoother between frames, and keeps
//                        going for a moment if vision drops out.
//               Either way the wheel odometry on /<agent>/odom still gives
//               Nav2 the robot's speed.
// /<agent>/orb_slam3/odom (nav_msgs/Odometry) is the SLAM pose of
// base_footprint; /<agent>/orb_slam3/scan (sensor_msgs/LaserScan) the live
// obstacle scan for Nav2's local costmap. /<agent>/odom is the wheel odometry.
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <opencv2/core.hpp>
#include <Eigen/Dense>

#include <System.h>
#include <KeyFrame.h>
#include <Map.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

namespace orbslam3_nav {

struct GridParams {
    double resolution        = 0.05;   // m per cell
    double maxRange          = 1.0;    // obstacles and free space only this close (m)
    double minHeight         = 0.03;   // obstacle band above the floor (m) ...
    double maxHeight         = 0.20;   // ... up to the robot's top plus a margin
    double belowFloor        = 0.03;   // points further below the floor are reflections
    int    minPoints         = 3;      // depth points needed to call a bin an obstacle
    double period            = 1.0;    // s between map renders
    double scanInterval      = 0.5;    // s between stored scans ...
    double scanMove          = 0.05;   // ... and only after moving this far (m)
    double scanTurnDeg       = 5.0;    // ... or turning this much
    int    pixelStride       = 3;      // depth pixels sampled every N in u and v
    double edgeJump          = 0.05;   // relative depth jump that marks a flying pixel
    double fovDeg            = 120.0;  // scan fan (wider than the camera is harmless)
    int    bins              = 120;
    double occThreshold      = 1.0;    // log-odds: >= occupied (needs two hits)
    double freeThreshold     = -0.3;   // log-odds: <= free
    double nominalCamHeight  = 0.05;   // sanity check for the floor fit (m)
    bool   mountFromParams   = false;  // skip the floor fit, use the three below
    double camHeight         = 0.05;   // m above the floor
    double camPitchDeg       = 0.0;    // + = looking down
    double camRollDeg        = 0.0;
    double cameraForward     = 0.21;   // camera ahead of the rear axle (m)
    double cameraLeft        = 0.0;    // camera left of the car's centre line (m)
    double scanRate          = 10.0;   // Hz of the live LaserScan
    bool   navFrames         = false;  // REP-105 tree for Nav2 (see top of file)
    std::string poseSource   = "slam"; // nav_frames: "slam" or "wheels" (see top of file)
    std::string globalFrame  = "map_nav";
    std::string baseFrame    = "base_footprint";
    std::string scanFrame    = "camera_floor";
    std::string odomFrame    = "odom";
    std::string cameraFrame  = "camera_color_optical_frame";
    std::string slamFrame    = "map";

    static GridParams declare(rclcpp::Node* n)
    {
        GridParams p;
        p.resolution       = n->declare_parameter("grid.resolution", p.resolution);
        p.maxRange         = n->declare_parameter("grid.max_range", p.maxRange);
        p.minHeight        = n->declare_parameter("grid.obstacle_min_height", p.minHeight);
        p.maxHeight        = n->declare_parameter("grid.obstacle_max_height", p.maxHeight);
        p.belowFloor       = n->declare_parameter("grid.below_floor", p.belowFloor);
        p.minPoints        = n->declare_parameter("grid.min_points", p.minPoints);
        p.period           = n->declare_parameter("grid.period", p.period);
        p.scanInterval     = n->declare_parameter("grid.scan_interval", p.scanInterval);
        p.scanMove         = n->declare_parameter("grid.scan_move", p.scanMove);
        p.scanTurnDeg      = n->declare_parameter("grid.scan_turn_deg", p.scanTurnDeg);
        p.pixelStride      = n->declare_parameter("grid.pixel_stride", p.pixelStride);
        p.edgeJump         = n->declare_parameter("grid.edge_jump", p.edgeJump);
        p.occThreshold     = n->declare_parameter("grid.occupied_logodds", p.occThreshold);
        p.freeThreshold    = n->declare_parameter("grid.free_logodds", p.freeThreshold);
        p.nominalCamHeight = n->declare_parameter("grid.nominal_camera_height", p.nominalCamHeight);
        p.mountFromParams  = n->declare_parameter("grid.mount_from_params", p.mountFromParams);
        p.camHeight        = n->declare_parameter("grid.camera_height", p.camHeight);
        p.camPitchDeg      = n->declare_parameter("grid.camera_pitch_deg", p.camPitchDeg);
        p.camRollDeg       = n->declare_parameter("grid.camera_roll_deg", p.camRollDeg);
        p.globalFrame      = n->declare_parameter("grid.global_frame", p.globalFrame);
        p.baseFrame        = n->declare_parameter("grid.base_frame", p.baseFrame);
        p.scanFrame        = n->declare_parameter("grid.scan_frame", p.scanFrame);
        p.odomFrame        = n->declare_parameter("grid.odom_frame", p.odomFrame);
        p.cameraForward    = n->declare_parameter("grid.camera_forward", p.cameraForward);
        p.cameraLeft       = n->declare_parameter("grid.camera_left", p.cameraLeft);
        p.scanRate         = n->declare_parameter("grid.scan_rate", p.scanRate);
        p.poseSource       = n->declare_parameter("grid.pose_source", p.poseSource);
        if (p.poseSource != "slam" && p.poseSource != "wheels") {
            RCLCPP_WARN(n->get_logger(), "[grid] pose_source '%s' unknown; using slam", p.poseSource.c_str());
            p.poseSource = "slam";
        }
        p.pixelStride      = std::max(1, p.pixelStride);
        p.bins             = std::max(8, p.bins);
        return p;
    }
};

class OccupancyMapper {
public:
    OccupancyMapper(rclcpp::Node* node, ORB_SLAM3::System* slam,
                    float fx, float fy, float cx, float cy,
                    const GridParams& p, const std::string& agent)
        : node_(node), slam_(slam), fx_(fx), fy_(fy), cx_(cx), cy_(cy), p_(p), rng_(7)
    {
        grid_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
            agent + "/map", rclcpp::QoS(1).reliable().transient_local());
        // /<agent>/odom is the base driver's wheel odometry; this is the SLAM pose.
        odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(agent + "/orb_slam3/odom", 10);
        scan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(
            agent + "/orb_slam3/scan", rclcpp::SensorDataQoS());
        static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*node_);
        if (p_.navFrames) tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
        if (p_.navFrames && p_.poseSource == "wheels") {
            // Wheel odometry of the base driver: odom -> base_footprint.
            wheel_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
                agent + "/odom", 50,
                [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { onWheelOdom(*m); });
            // map_nav -> odom, re-sent at 20 Hz with the current time so TF
            // lookups never run past it while tracking is briefly lost.
            corr_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                                   [this] { publishCorrection(); });
        }

        if (p_.mountFromParams) {
            const float pi = (float)(p_.camPitchDeg * M_PI / 180.0);
            const float ro = (float)(p_.camRollDeg * M_PI / 180.0);
            const Eigen::Vector3f n(std::sin(ro) * std::cos(pi), -std::cos(ro) * std::cos(pi),
                                    -std::sin(pi));
            setMount(n, (float)p_.camHeight, "from parameters");
        } else {
            RCLCPP_INFO(node_->get_logger(),
                        "[grid] calibrating the floor from depth; keep the floor in view");
        }
        worker_ = std::thread(&OccupancyMapper::run, this);
    }

    ~OccupancyMapper()
    {
        stop_ = true;
        cv_.notify_all();
        if (worker_.joinable()) worker_.join();
    }

    // Tracking thread, once per frame, right after Track*: the reference
    // keyframe read here belongs to this frame.
    void addFrame(const cv::Mat& depthM, const Sophus::SE3f& Tcw, bool trackingOk,
                  const rclcpp::Time& stamp)
    {
        if (depthM.empty() || depthM.type() != CV_32F) return;
        if (!calibrated_) { tryCalibrate(depthM); return; }
        const double t = stamp.seconds();

        // The live scan needs only the floor calibration, not tracking.
        std::unique_ptr<Scan> scan;
        if (p_.scanRate > 0.0 && t - lastLiveT_ >= 1.0 / p_.scanRate) {
            scan = buildScan(depthM);
            if (scan) publishLiveScan(*scan, stamp);
            lastLiveT_ = t;
        }

        if (!trackingOk) return;
        ORB_SLAM3::KeyFrame* ref = slam_->GetTrackingReferenceKF();
        if (!ref) return;

        const Sophus::SE3f Twc = Tcw.inverse();
        ORB_SLAM3::Map* pMain = slam_->GetMainMap();
        if (pMain && ref->GetMap() == pMain) {
            publishOdom(Twc, stamp);
            if (p_.navFrames) {
                if (p_.poseSource == "slam") publishSlamBase(Twc, stamp);
                else updateCorrection(Twc, t);
            }
        }

        if (haveLast_) {
            if (t - lastT_ < p_.scanInterval) return;
            const float moved  = (Twc.translation() - lastTwc_.translation()).norm();
            const float turned = (lastTwc_.so3().inverse() * Twc.so3()).log().norm();
            if (moved < p_.scanMove && turned < p_.scanTurnDeg * M_PI / 180.0) return;
        }
        if (!scan) scan = buildScan(depthM);
        if (!scan) return;
        scan->anchor = ref;
        scan->Tcr    = Tcw * ref->GetPose().inverse();   // this camera relative to its keyframe
        {
            std::lock_guard<std::mutex> lk(scans_mx_);
            scans_.push_back(std::move(scan));
        }
        lastT_ = t; lastTwc_ = Twc; haveLast_ = true;
    }

private:
    struct Scan {
        ORB_SLAM3::KeyFrame* anchor = nullptr;
        Sophus::SE3f Tcr;                 // Tcw(frame) = Tcr * Tcw(anchor)
        std::vector<float>   len;         // per bin: free length (m), <0 unknown
        std::vector<uint8_t> hit;         // per bin: an obstacle ends the ray
    };

    // ── floor calibration ─────────────────────────────────────────────────
    struct Plane { Eigen::Vector3f n; float d; };   // n.p + d = 0, n up, d = camera height

    bool fitFloor(const cv::Mat& depth, Plane& out)
    {
        std::vector<Eigen::Vector3f> pts;
        pts.reserve(6000);
        for (int v = 0; v < depth.rows; v += 8) {
            const float* row = depth.ptr<float>(v);
            for (int u = 0; u < depth.cols; u += 8) {
                const float z = row[u];
                if (!(z > 0.15f && z < 1.5f)) continue;
                pts.emplace_back((u - cx_) * z / fx_, (v - cy_) * z / fy_, z);
            }
        }
        if (pts.size() < 200) { why_ = "too little depth between 0.15 and 1.5 m"; return false; }

        // Camera "up" is -y in the optical frame; the floor normal must be
        // within 45 deg of it and the camera 1-30 cm above the plane.
        const Eigen::Vector3f up(0.f, -1.f, 0.f);
        const float cosMax = std::cos(45.f * (float)M_PI / 180.f);
        std::uniform_int_distribution<size_t> pick(0, pts.size() - 1);
        int best = 0;
        Plane bp{up, 0.f};
        for (int it = 0; it < 300; ++it) {
            const Eigen::Vector3f& a = pts[pick(rng_)];
            const Eigen::Vector3f& b = pts[pick(rng_)];
            const Eigen::Vector3f& c = pts[pick(rng_)];
            Eigen::Vector3f n = (b - a).cross(c - a);
            const float len = n.norm();
            if (len < 1e-6f) continue;
            n /= len;
            if (n.dot(up) < 0.f) n = -n;
            const float d = -n.dot(a);
            if (n.dot(up) < cosMax || d < 0.01f || d > 0.30f) continue;
            int cnt = 0;
            for (const auto& p : pts)
                if (std::fabs(n.dot(p) + d) < 0.01f) ++cnt;
            if (cnt > best) { best = cnt; bp = {n, d}; }
        }
        if (best < 150) {
            why_ = "no floor in view (need a level surface 1-30 cm below the camera, "
                   "within 45 deg of the camera's horizon)";
            return false;
        }

        // Least-squares refinement on the inliers.
        Eigen::Vector3f c = Eigen::Vector3f::Zero();
        std::vector<Eigen::Vector3f> in;
        for (const auto& p : pts)
            if (std::fabs(bp.n.dot(p) + bp.d) < 0.01f) { in.push_back(p); c += p; }
        c /= (float)in.size();
        Eigen::Matrix3f C = Eigen::Matrix3f::Zero();
        for (const auto& p : in) C += (p - c) * (p - c).transpose();
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(C);
        Eigen::Vector3f n = es.eigenvectors().col(0);
        if (n.dot(up) < 0.f) n = -n;
        const float d = -n.dot(c);
        if (d < 0.01f || d > 0.30f) { why_ = "floor candidate rejected after refinement"; return false; }
        out = {n.normalized(), d};
        return true;
    }

    void tryCalibrate(const cv::Mat& depth)
    {
        if (++calib_frame_ % 3 != 0) return;          // every third frame is plenty
        // Say why we are still waiting, every 5 s, so a missing map_nav frame
        // is never a mystery.
        auto status = [&](const std::string& msg) {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "[grid] still calibrating the floor: %s", msg.c_str());
        };
        Plane pl;
        if (!fitFloor(depth, pl)) { status(why_); return; }
        fits_.push_back(pl);
        if (fits_.size() > 15) fits_.pop_front();
        if (fits_.size() < 15) {
            status("floor found " + std::to_string(fits_.size()) + "/15 times (height " +
                   std::to_string(pl.d).substr(0, 5) + " m); keep the camera still");
            return;
        }

        // Component-wise median, then keep the fits that agree with it.
        auto median = [](std::vector<float> v) {
            std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
            return v[v.size() / 2];
        };
        std::vector<float> nx, ny, nz, dd;
        for (const auto& f : fits_) { nx.push_back(f.n.x()); ny.push_back(f.n.y());
                                      nz.push_back(f.n.z()); dd.push_back(f.d); }
        const Eigen::Vector3f nm = Eigen::Vector3f(median(nx), median(ny), median(nz)).normalized();
        const float dm = median(dd);
        Eigen::Vector3f ns = Eigen::Vector3f::Zero();
        float ds = 0.f;
        int ok = 0;
        for (const auto& f : fits_) {
            const float ang = std::acos(std::clamp(f.n.dot(nm), -1.f, 1.f)) * 180.f / (float)M_PI;
            if (ang < 2.f && std::fabs(f.d - dm) < 0.01f) { ns += f.n; ds += f.d; ++ok; }
        }
        if (ok < 12) {                                // not settled yet; keep sampling
            status("floor fits do not agree yet (" + std::to_string(ok) +
                   "/15 within 2 deg and 1 cm); keep the camera still");
            return;
        }
        setMount(ns.normalized(), ds / ok, "fitted to the floor");
    }

    // Level base frame under the camera: z = floor normal, x = camera forward
    // projected onto the floor, origin on the floor below the camera.
    void setMount(const Eigen::Vector3f& nUp, float h, const char* how)
    {
        const Eigen::Vector3f z = nUp.normalized();
        Eigen::Vector3f x = Eigen::Vector3f(0.f, 0.f, 1.f);
        x = (x - x.dot(z) * z).normalized();
        const Eigen::Vector3f y = z.cross(x);
        Eigen::Matrix3f R;
        R.col(0) = x; R.col(1) = y; R.col(2) = z;     // base axes in camera coordinates
        Eigen::Quaternionf q(R);
        q.normalize();
        T_cam_sensor_ = Sophus::SE3f(q, -h * z);              // camera_floor
        T_sensor_cam_ = T_cam_sensor_.inverse();
        const Sophus::SE3f T_sensor_base(Eigen::Quaternionf::Identity(),
            Eigen::Vector3f(-(float)p_.cameraForward, -(float)p_.cameraLeft, 0.f));
        T_cam_base_ = T_cam_sensor_ * T_sensor_base;           // base_footprint (rear axle)
        T_base_cam_ = T_cam_base_.inverse();

        const float pitch = std::asin(std::clamp(-z.z(), -1.f, 1.f)) * 180.f / (float)M_PI;
        const float roll  = std::atan2(z.x(), -z.y()) * 180.f / (float)M_PI;
        RCLCPP_INFO(node_->get_logger(),
                    "[grid] camera mount %s: height %.3f m, pitch %.1f deg (+ = down), roll %.1f deg",
                    how, h, pitch, roll);
        if (!p_.mountFromParams && std::fabs(h - p_.nominalCamHeight) > 0.03)
            RCLCPP_WARN(node_->get_logger(),
                        "[grid] fitted camera height %.3f m is far from the expected %.3f m "
                        "(grid.nominal_camera_height): was the floor in view?",
                        h, p_.nominalCamHeight);
        publishStaticFrames();
        calibrated_ = true;
    }

    void publishStaticFrames()
    {
        // ORB-SLAM3 world = first camera (optical axes); the node's "map"
        // frame is that world with ROS axes. map_nav = base frame under the
        // first camera pose.
        Eigen::Matrix3f Ro2r;
        Ro2r << 0, 0, 1,  -1, 0, 0,  0, -1, 0;
        const Sophus::SE3f T_map_orb(Eigen::Quaternionf(Ro2r).normalized(), Eigen::Vector3f::Zero());
        const Sophus::SE3f T_map_nav = T_map_orb * T_cam_base_;
        std::vector<geometry_msgs::msg::TransformStamped> v;
        v.push_back(toTf(T_map_nav, p_.slamFrame, p_.globalFrame));
        if (p_.navFrames && p_.poseSource == "slam")   // the SLAM pose is odom -> base_footprint
            v.push_back(toTf(Sophus::SE3f(), p_.globalFrame, p_.odomFrame));
        if (p_.navFrames) {          // camera hangs under the robot
            v.push_back(toTf(T_base_cam_, p_.baseFrame, p_.cameraFrame));
            v.push_back(toTf(T_base_cam_ * T_cam_sensor_, p_.baseFrame, p_.scanFrame));
        } else {                     // robot hangs under the SLAM camera pose
            v.push_back(toTf(T_cam_base_, p_.cameraFrame, p_.baseFrame));
            v.push_back(toTf(T_cam_sensor_, p_.cameraFrame, p_.scanFrame));
        }
        static_tf_->sendTransform(v);
    }

    geometry_msgs::msg::TransformStamped toTf(const Sophus::SE3f& T, const std::string& parent,
                                              const std::string& child)
    {
        geometry_msgs::msg::TransformStamped m;
        m.header.stamp = node_->now();
        m.header.frame_id = parent;
        m.child_frame_id = child;
        const Eigen::Quaternionf q = T.unit_quaternion();
        m.transform.translation.x = T.translation().x();
        m.transform.translation.y = T.translation().y();
        m.transform.translation.z = T.translation().z();
        m.transform.rotation.x = q.x(); m.transform.rotation.y = q.y();
        m.transform.rotation.z = q.z(); m.transform.rotation.w = q.w();
        return m;
    }

    // ── per-frame outputs ─────────────────────────────────────────────────
    void publishOdom(const Sophus::SE3f& Twc, const rclcpp::Time& stamp)
    {
        const Sophus::SE3f Tnb = T_base_cam_ * Twc * T_cam_base_;
        const Eigen::Matrix3f R = Tnb.rotationMatrix();
        const float yaw = std::atan2(R(1, 0), R(0, 0));
        nav_msgs::msg::Odometry o;
        o.header.stamp = stamp;
        o.header.frame_id = p_.globalFrame;
        o.child_frame_id = p_.baseFrame;
        const Eigen::Quaternionf q = Tnb.unit_quaternion();
        o.pose.pose.position.x = Tnb.translation().x();
        o.pose.pose.position.y = Tnb.translation().y();
        o.pose.pose.position.z = Tnb.translation().z();
        o.pose.pose.orientation.x = q.x(); o.pose.pose.orientation.y = q.y();
        o.pose.pose.orientation.z = q.z(); o.pose.pose.orientation.w = q.w();
        const double t = stamp.seconds();
        if (haveOdom_ && t > odomT_) {
            const double dt = t - odomT_;
            const Eigen::Vector3f d = Tnb.translation() - odomP_;
            o.twist.twist.linear.x  = (std::cos(yaw) * d.x() + std::sin(yaw) * d.y()) / dt;
            o.twist.twist.linear.y  = (-std::sin(yaw) * d.x() + std::cos(yaw) * d.y()) / dt;
            o.twist.twist.angular.z = std::remainder(yaw - odomYaw_, 2.0 * M_PI) / dt;
        }
        odom_pub_->publish(o);
        odomT_ = t; odomP_ = Tnb.translation(); odomYaw_ = yaw; haveOdom_ = true;
    }

    // ── live scan and navigation frames ───────────────────────────────────
    void publishLiveScan(const Scan& s, const rclcpp::Time& stamp)
    {
        const float fov  = (float)(p_.fovDeg * M_PI / 180.0);
        const float binW = fov / p_.bins;
        sensor_msgs::msg::LaserScan m;
        m.header.stamp = stamp;
        m.header.frame_id = p_.scanFrame;
        m.angle_min = -0.5f * fov + 0.5f * binW;
        m.angle_increment = binW;
        m.angle_max = m.angle_min + binW * (p_.bins - 1);
        m.range_min = 0.05f;
        m.range_max = (float)p_.maxRange;
        m.ranges.resize(p_.bins);
        // An obstacle is a range; a direction seen free to the full range is
        // +inf (costmaps clear it); anything else carries no information.
        const float nan = std::numeric_limits<float>::quiet_NaN();
        for (int b = 0; b < p_.bins; ++b) {
            if (s.len[b] < 0.f) m.ranges[b] = nan;
            else if (s.hit[b]) m.ranges[b] = s.len[b];
            else m.ranges[b] = s.len[b] >= 0.9f * (float)p_.maxRange
                             ? std::numeric_limits<float>::infinity() : nan;
        }
        scan_pub_->publish(m);
    }

    struct Pose2D { double t, x, y, yaw; };

    void onWheelOdom(const nav_msgs::msg::Odometry& m)
    {
        const auto& q = m.pose.pose.orientation;
        const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        std::lock_guard<std::mutex> lk(nav_mx_);
        wheel_.push_back({rclcpp::Time(m.header.stamp).seconds(), m.pose.pose.position.x,
                          m.pose.pose.position.y, yaw});
        while (wheel_.size() > 400) wheel_.pop_front();
    }

    // Wheel pose at time t (interpolated; the newest one if t is up to 0.2 s
    // past it). False if there is no wheel odometry around t.
    bool wheelAt(double t, Pose2D& out)
    {
        std::lock_guard<std::mutex> lk(nav_mx_);
        if (wheel_.empty() || t < wheel_.front().t) return false;
        if (t >= wheel_.back().t) {
            if (t - wheel_.back().t > 0.2) return false;
            out = wheel_.back();
            return true;
        }
        for (size_t i = 1; i < wheel_.size(); ++i) {
            if (wheel_[i].t < t) continue;
            const Pose2D& a = wheel_[i - 1];
            const Pose2D& b = wheel_[i];
            const double f = (b.t > a.t) ? (t - a.t) / (b.t - a.t) : 0.0;
            out = {t, a.x + f * (b.x - a.x), a.y + f * (b.y - a.y),
                   a.yaw + f * std::remainder(b.yaw - a.yaw, 2.0 * M_PI)};
            return true;
        }
        return false;
    }

    // map_nav -> odom = (SLAM pose of base_footprint) * (wheel pose)^-1, in
    // the plane: Nav2 is 2D, and a level map_nav keeps odom level too.
    void updateCorrection(const Sophus::SE3f& Twc, double t)
    {
        Pose2D w;
        if (!wheelAt(t, w)) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "[grid] no wheel odometry on %s/odom near the camera time: map_nav -> odom not "
                "updated (is the base driver running?)", node_->get_name());
            return;
        }
        const Sophus::SE3f Tnb = T_base_cam_ * Twc * T_cam_base_;
        const Eigen::Matrix3f R = Tnb.rotationMatrix();
        const double sx = Tnb.translation().x(), sy = Tnb.translation().y();
        const double syaw = std::atan2(R(1, 0), R(0, 0));
        const double cyaw = std::remainder(syaw - w.yaw, 2.0 * M_PI);
        const double c = std::cos(cyaw), s = std::sin(cyaw);
        std::lock_guard<std::mutex> lk(nav_mx_);
        corr_ = {t, sx - (c * w.x - s * w.y), sy - (s * w.x + c * w.y), cyaw};
        haveCorr_ = true;
    }

    // pose_source slam: odom (= map_nav) -> base_footprint is the SLAM pose of
    // the rear axle, in the plane (Nav2 is 2D and keeps base_footprint on the
    // floor), stamped with the camera frame's time.
    void publishSlamBase(const Sophus::SE3f& Twc, const rclcpp::Time& stamp)
    {
        const Sophus::SE3f Tnb = T_base_cam_ * Twc * T_cam_base_;
        const Eigen::Matrix3f R = Tnb.rotationMatrix();
        const double yaw = std::atan2(R(1, 0), R(0, 0));
        geometry_msgs::msg::TransformStamped m;
        m.header.stamp = stamp;
        m.header.frame_id = p_.odomFrame;
        m.child_frame_id = p_.baseFrame;
        m.transform.translation.x = Tnb.translation().x();
        m.transform.translation.y = Tnb.translation().y();
        m.transform.rotation.z = std::sin(0.5 * yaw);
        m.transform.rotation.w = std::cos(0.5 * yaw);
        tf_->sendTransform(m);
    }

    void publishCorrection()
    {
        Pose2D c;
        {
            std::lock_guard<std::mutex> lk(nav_mx_);
            if (!haveCorr_) return;
            c = corr_;
        }
        geometry_msgs::msg::TransformStamped m;
        m.header.stamp = node_->now();
        m.header.frame_id = p_.globalFrame;
        m.child_frame_id = p_.odomFrame;
        m.transform.translation.x = c.x;
        m.transform.translation.y = c.y;
        m.transform.rotation.z = std::sin(0.5 * c.yaw);
        m.transform.rotation.w = std::cos(0.5 * c.yaw);
        tf_->sendTransform(m);
    }

    std::unique_ptr<Scan> buildScan(const cv::Mat& depth)
    {
        const int B = p_.bins;
        const float fov  = (float)(p_.fovDeg * M_PI / 180.0);
        const float binW = fov / B;
        const float maxR = (float)p_.maxRange;
        std::vector<std::vector<float>> band(B);
        std::vector<float> freeTo(B, -1.f);
        const Eigen::Matrix3f Rbc = T_sensor_cam_.rotationMatrix();   // scans live in camera_floor
        const Eigen::Vector3f tbc = T_sensor_cam_.translation();
        const int st = p_.pixelStride;
        const float jumpRel = (float)p_.edgeJump;

        auto at = [&](int v, int u) { return depth.ptr<float>(v)[u]; };
        auto jumps = [&](int v, int u, float z) {
            if (v < 0 || u < 0 || v >= depth.rows || u >= depth.cols) return false;
            const float zn = at(v, u);
            return zn > 0.f && std::fabs(zn - z) > jumpRel * z;
        };

        for (int v = 0; v < depth.rows; v += st) {
            for (int u = 0; u < depth.cols; u += st) {
                const float z = at(v, u);
                if (!(z > 0.1f) || z > 3.0f) continue;
                // Flying pixels sit on depth edges: skip anything whose
                // neighbours jump in depth.
                if (jumps(v, u - st, z) || jumps(v, u + st, z) ||
                    jumps(v - st, u, z) || jumps(v + st, u, z)) continue;
                const Eigen::Vector3f pc((u - cx_) * z / fx_, (v - cy_) * z / fy_, z);
                const Eigen::Vector3f pb = Rbc * pc + tbc;
                const float h = pb.z();
                if (h < -p_.belowFloor || h > p_.maxHeight) continue;
                const float r = std::hypot(pb.x(), pb.y());
                if (r < 0.05f) continue;
                const int bi = (int)((std::atan2(pb.y(), pb.x()) + 0.5f * fov) / binW);
                if (bi < 0 || bi >= B) continue;
                if (h >= p_.minHeight) {
                    if (r <= maxR) band[bi].push_back(r);
                    else freeTo[bi] = maxR;               // obstacle beyond range: free up to it
                } else {
                    freeTo[bi] = std::max(freeTo[bi], std::min(r, maxR));   // floor seen this far
                }
            }
        }

        auto s = std::make_unique<Scan>();
        s->len.assign(B, -1.f);
        s->hit.assign(B, 0);
        int known = 0;
        const int need = std::max(1, p_.minPoints);
        for (int b = 0; b < B; ++b) {
            auto& r = band[b];
            float hit = -1.f;
            if ((int)r.size() >= need) {
                // Nearest range backed by `need` points within 5 cm: a lone
                // noisy pixel never makes an obstacle.
                std::sort(r.begin(), r.end());
                for (size_t i = 0; i + need - 1 < r.size(); ++i)
                    if (r[i + need - 1] - r[i] <= 0.05f) { hit = r[i]; break; }
            }
            if (hit > 0.f) { s->len[b] = hit; s->hit[b] = 1; ++known; }
            else if (freeTo[b] > 0.f) { s->len[b] = freeTo[b]; ++known; }
        }
        if (!known) return nullptr;
        return s;
    }

    // ── map rendering (own thread) ────────────────────────────────────────
    void run()
    {
        while (!stop_) {
            {
                std::unique_lock<std::mutex> lk(run_mx_);
                cv_.wait_for(lk, std::chrono::duration<double>(p_.period),
                             [this] { return stop_.load(); });
            }
            if (stop_) break;
            if (calibrated_) render();
        }
    }

    void render()
    {
        ORB_SLAM3::Map* pMain = slam_->GetMainMap();
        if (!pMain) return;
        std::vector<std::shared_ptr<const Scan>> scans;
        {
            std::lock_guard<std::mutex> lk(scans_mx_);
            scans = scans_;
        }
        if (scans.empty()) return;

        // Each scan's base pose in map_nav, from its keyframe's CURRENT pose.
        // A culled keyframe is followed up its parents, as ORB-SLAM3 does
        // when it saves trajectories.
        struct Pose2 { float x, y, yaw; const Scan* s; };
        std::vector<Pose2> poses;
        poses.reserve(scans.size());
        for (const auto& s : scans) {
            ORB_SLAM3::KeyFrame* k = s->anchor;
            Sophus::SE3f Trw;
            for (int guard = 0; k && k->isBad() && guard < 1000; ++guard) {
                Trw = Trw * k->mTcp;
                k = k->GetParent();
            }
            if (!k || k->isBad() || k->GetMap() != pMain) continue;   // not in the main map (yet)
            const Sophus::SE3f Tcw = s->Tcr * Trw * k->GetPose();
            const Sophus::SE3f Tnb = T_base_cam_ * Tcw.inverse() * T_cam_sensor_;   // camera_floor in map_nav
            const Eigen::Matrix3f R = Tnb.rotationMatrix();
            poses.push_back({Tnb.translation().x(), Tnb.translation().y(),
                             std::atan2(R(1, 0), R(0, 0)), s.get()});
        }
        if (poses.empty()) return;

        const float res  = (float)p_.resolution;
        const float pad  = (float)p_.maxRange + 2.f * res;
        float minx = 1e9f, miny = 1e9f, maxx = -1e9f, maxy = -1e9f;
        for (const auto& p : poses) {
            minx = std::min(minx, p.x); maxx = std::max(maxx, p.x);
            miny = std::min(miny, p.y); maxy = std::max(maxy, p.y);
        }
        minx -= pad; miny -= pad; maxx += pad; maxy += pad;
        const int W = (int)std::ceil((maxx - minx) / res);
        const int H = (int)std::ceil((maxy - miny) / res);
        if (W <= 0 || H <= 0 || (long)W * H > 25000000L) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                                 "[grid] map would be %d x %d cells; not rendered", W, H);
            return;
        }

        const float lHit = 0.85f, lMiss = -0.4f, lMin = -2.f, lMax = 3.5f;
        std::vector<float> L((size_t)W * H, 0.f);
        std::vector<int> hitStamp((size_t)W * H, -1), missStamp((size_t)W * H, -1);
        auto cell = [&](float x, float y) -> long {
            const int ix = (int)std::floor((x - minx) / res);
            const int iy = (int)std::floor((y - miny) / res);
            if (ix < 0 || iy < 0 || ix >= W || iy >= H) return -1;
            return (long)iy * W + ix;
        };
        const int B = p_.bins;
        const float fov  = (float)(p_.fovDeg * M_PI / 180.0);
        const float binW = fov / B;
        const float step = 0.5f * res;
        std::vector<long> hits;
        for (int si = 0; si < (int)poses.size(); ++si) {
            const Pose2& ps = poses[si];
            const Scan& s = *ps.s;
            hits.clear();
            // Obstacle cells first, so no ray of this scan clears them.
            for (int b = 0; b < B; ++b) {
                if (s.len[b] < 0.f || !s.hit[b]) continue;
                const float a = ps.yaw - 0.5f * fov + (b + 0.5f) * binW;
                const long c = cell(ps.x + s.len[b] * std::cos(a), ps.y + s.len[b] * std::sin(a));
                if (c >= 0 && hitStamp[c] != si) { hitStamp[c] = si; hits.push_back(c); }
            }
            for (int b = 0; b < B; ++b) {
                if (s.len[b] < 0.f) continue;
                const float a = ps.yaw - 0.5f * fov + (b + 0.5f) * binW;
                const float ca = std::cos(a), sa = std::sin(a);
                const int n = (int)(s.len[b] / step);
                long last = -2;
                for (int k = 0; k <= n; ++k) {
                    const long c = cell(ps.x + k * step * ca, ps.y + k * step * sa);
                    if (c < 0 || c == last) continue;
                    last = c;
                    if (hitStamp[c] == si || missStamp[c] == si) continue;
                    missStamp[c] = si;
                    L[c] = std::max(lMin, L[c] + lMiss);
                }
            }
            for (long c : hits) L[c] = std::min(lMax, L[c] + lHit);
        }

        nav_msgs::msg::OccupancyGrid g;
        g.header.stamp = node_->now();
        g.header.frame_id = p_.globalFrame;
        g.info.map_load_time = g.header.stamp;
        g.info.resolution = res;
        g.info.width = W;
        g.info.height = H;
        g.info.origin.position.x = minx;
        g.info.origin.position.y = miny;
        g.info.origin.orientation.w = 1.0;
        g.data.resize((size_t)W * H);
        for (size_t i = 0; i < L.size(); ++i)
            g.data[i] = L[i] >= p_.occThreshold ? 100 : (L[i] <= p_.freeThreshold ? 0 : -1);
        grid_pub_->publish(g);
    }

    rclcpp::Node* node_;
    ORB_SLAM3::System* slam_;
    const float fx_, fy_, cx_, cy_;
    const GridParams p_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;

    // Navigation frames (nav_frames only).
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
    rclcpp::TimerBase::SharedPtr corr_timer_;
    std::mutex nav_mx_;
    std::deque<Pose2D> wheel_;
    Pose2D corr_{0, 0, 0, 0};
    bool haveCorr_ = false;
    double lastLiveT_ = -1e9;

    // Mount (written once by the tracking thread before calibrated_ is set):
    // camera_floor (scans) and base_footprint (rear axle).
    Sophus::SE3f T_cam_sensor_, T_sensor_cam_, T_cam_base_, T_base_cam_;
    std::atomic<bool> calibrated_{false};
    std::deque<Plane> fits_;
    std::string why_;
    int calib_frame_ = 0;
    std::mt19937 rng_;

    // Tracking-thread state.
    bool haveLast_ = false;
    double lastT_ = 0.0;
    Sophus::SE3f lastTwc_;
    bool haveOdom_ = false;
    double odomT_ = 0.0, odomYaw_ = 0.0;
    Eigen::Vector3f odomP_ = Eigen::Vector3f::Zero();

    std::mutex scans_mx_;
    std::vector<std::shared_ptr<const Scan>> scans_;

    std::atomic<bool> stop_{false};
    std::mutex run_mx_;
    std::condition_variable cv_;
    std::thread worker_;
};

}  // namespace orbslam3_nav
