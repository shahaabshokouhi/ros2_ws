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
//               Safety gate: Nav2's velocity commands come in on
//               /<agent>/cmd_vel_gate_in and leave on /<agent>/cmd_vel only
//               while the SLAM pose is fresh (pose_source slam). TF hands out
//               the last pose however old it is, so after a tracking loss
//               Nav2 would keep steering on a frozen position (seen in
//               simulation: the car drove on to 11 cm from a wall). On a
//               tracking loss the gate sends a few stops, cancels the Nav2
//               goal and then stays silent: a stream of stops would cancel
//               out the keyboard teleop, which publishes on /<agent>/cmd_vel
//               directly (not gated) and is how the car is driven back.
//               Provisional maps (nav_frames, grid.bridge, on by default):
//               after a tracking loss ORB-SLAM3 starts a new map with its own
//               origin. Instead of holding the car until that map merges back,
//               the new map is placed in the main map by the wheel odometry
//               across the gap:
//                 T_main<-new = T_main,ca * T_ca,cb * T_new,cb^-1
//               ca = the last well-tracked frame before the loss (through its
//               keyframe, so it follows later refinement), cb = the new map's
//               first keyframe (its origin; fixed in bundle adjustment), and
//               T_ca,cb = T_cam_base * W(ta)^-1 * W(tb) * T_base_cam from the
//               base's odometry W. Heading across the gap: grid.bridge_heading
//               "imu" (the motor board's IMU yaw, the heading the jetracer
//               package's EKF uses; position steps from the wheels) or
//               "wheels". The new map is used once it has
//               grid.bridge_min_points map points and has tracked for
//               grid.bridge_min_time s. Then the robot's pose, the grid and
//               Nav2 carry on in map_nav as before, with no time or distance
//               limit, until ORB-SLAM3 merges the map into the main map. Its
//               scans form a second grid layer that only fills cells the main
//               map has not seen; at the merge they get their true poses. A
//               loss inside a provisional map chains (new <- provisional <-
//               main). If the odometry jumped or stopped during the gap (motor
//               board reconnected), the new map is not placed (the car is held
//               as before). Keyframes of a provisional map are not shared
//               with other robots (MA.cc shares the main map only) until the
//               merge. /<agent>/orb_slam3/provisional (Bool, latched) says
//               whether the pose is provisional; the merge logs how far off
//               the odometry placement was, for both headings.
// /<agent>/orb_slam3/odom (nav_msgs/Odometry) is the SLAM pose of
// base_footprint; /<agent>/orb_slam3/scan (sensor_msgs/LaserScan) the live
// obstacle scan for Nav2's local costmap. /<agent>/odom is the wheel odometry.
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
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
#include <cstdio>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

namespace orbslam3_nav {

struct GridParams {
    double resolution        = 0.05;   // m per cell
    double maxRange          = 2.0;    // obstacles and free space only this close (m; 1.0 until 13 Oct 2026)
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
    double mapHalfSize       = 0.0;    // the map covers at least +-this around the start (m);
                                       // with Nav2 treating unknown as free, goals anywhere
                                       // in it are plannable from the first second
    double mapMargin         = 2.0;    // unknown margin around what was seen (m), so
                                       // Nav2 goals just past the explored area are on the map
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
    // Provisional maps (nav_frames; see top of file).
    bool   bridge            = true;   // place a new SLAM map in the main map by odometry
    int    bridgeMinPoints   = 300;    // the new map is used with this many map points ...
    double bridgeMinTime     = 1.0;    // ... once it has tracked this long without a break (s)
    std::string bridgeHeading = "imu"; // heading across the gap: "imu" or "wheels"
    int    anchorMinPoints   = 100;    // tracked points the anchor frame before a loss should have
    // The launch started the base driver (jetracer): its heartbeat must come. Without
    // this (simulation) a missing heartbeat is normal and only a seen one is checked.
    bool   expectDriver      = false;

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
        p.mapMargin        = n->declare_parameter("grid.map_margin", p.mapMargin);
        p.mapHalfSize      = n->declare_parameter("grid.map_half_size", p.mapHalfSize);
        p.cameraForward    = n->declare_parameter("grid.camera_forward", p.cameraForward);
        p.cameraLeft       = n->declare_parameter("grid.camera_left", p.cameraLeft);
        p.scanRate         = n->declare_parameter("grid.scan_rate", p.scanRate);
        p.poseSource       = n->declare_parameter("grid.pose_source", p.poseSource);
        if (p.poseSource != "slam" && p.poseSource != "wheels") {
            RCLCPP_WARN(n->get_logger(), "[grid] pose_source '%s' unknown; using slam", p.poseSource.c_str());
            p.poseSource = "slam";
        }
        p.bridge           = n->declare_parameter("grid.bridge", p.bridge);
        p.bridgeMinPoints  = n->declare_parameter("grid.bridge_min_points", p.bridgeMinPoints);
        p.bridgeMinTime    = n->declare_parameter("grid.bridge_min_time", p.bridgeMinTime);
        p.bridgeHeading    = n->declare_parameter("grid.bridge_heading", p.bridgeHeading);
        p.anchorMinPoints  = n->declare_parameter("grid.bridge_anchor_min_points", p.anchorMinPoints);
        p.expectDriver     = n->declare_parameter("grid.expect_driver", p.expectDriver);
        if (p.bridgeHeading != "imu" && p.bridgeHeading != "wheels") {
            RCLCPP_WARN(n->get_logger(), "[grid] bridge_heading '%s' unknown; using imu", p.bridgeHeading.c_str());
            p.bridgeHeading = "imu";
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
        if (p_.navFrames) {
            tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
            cmd_out_ = node_->create_publisher<geometry_msgs::msg::Twist>(agent + "/cmd_vel", 10);
            cmd_in_ = node_->create_subscription<geometry_msgs::msg::Twist>(
                agent + "/cmd_vel_gate_in", 10,
                [this](geometry_msgs::msg::Twist::ConstSharedPtr m) { gateCommand(*m); });
            for (const char* action : {"navigate_to_pose", "navigate_through_poses"})
                cancel_.push_back(node_->create_client<action_msgs::srv::CancelGoal>(
                    agent + "/" + action + "/_action/cancel_goal"));
            // The base driver's heartbeat (jetracer, 10 Hz). Checked once it has
            // been seen, or from 5 s after the start when the launch started the
            // driver (grid.expect_driver): in simulation there is none.
            driverStart_ = std::chrono::steady_clock::now();
            driver_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
                agent + "/base_driver/ok", 10, [this](std_msgs::msg::Bool::ConstSharedPtr m) {
                    driverOk_ = m->data;
                    driverWall_ = std::chrono::steady_clock::now();
                    driverSeen_ = true;
                });
        }
        if (p_.navFrames && (p_.poseSource == "wheels" || p_.bridge)) {
            // Wheel odometry of the base driver: odom -> base_footprint
            // (pose source wheels), and the gap across a tracking loss.
            wheel_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
                agent + "/odom", 50,
                [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { onWheelOdom(*m); });
        }
        if (bridgeOn()) {
            // The motor board's IMU heading (jetracer); none in simulation,
            // where the bridge then uses the wheel heading.
            imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
                agent + "/imu", 50,
                [this](sensor_msgs::msg::Imu::ConstSharedPtr m) { onImu(*m); });
            prov_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
                agent + "/orb_slam3/provisional", rclcpp::QoS(1).reliable().transient_local());
            std_msgs::msg::Bool b;
            b.data = false;
            prov_pub_->publish(b);
        }
        if (p_.navFrames && p_.poseSource == "wheels") {
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

        if (!trackingOk) {                                // safety gate: a real loss
            slamLost_ = true;
            if (bridgeOn()) freezeAnchor();
            return;
        }
        ORB_SLAM3::KeyFrame* ref = slam_->GetTrackingReferenceKF();
        if (!ref) return;

        Sophus::SE3f Twc = Tcw.inverse();   // in the map the robot is in; below, in the main map
        ORB_SLAM3::Map* cur = ref->GetMap();
        if (!firstMap_) firstMap_ = cur;
        ORB_SLAM3::Map* pMain = navMap();
        const bool inMain = pMain && cur == pMain;
        bool placed = inMain;
        if (bridgeOn() && pMain && cur) {
            if (!inMain) {
                std::string why;
                Sophus::SE3f Tmx;
                if (bridgeFrame(cur, pMain, t, Tmx, why)) {
                    Twc = Tmx * Twc;
                    placed = true;
                } else {
                    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                        "[nav] SLAM is in a new map after a tracking loss: no robot pose for Nav2 "
                        "yet (%s)", why.c_str());
                }
            }
            if (placed && !poseConfirmed(t, Twc)) return;     // a jump: wait for the next frame
            if (inMain) {
                bridgeBackInMain(pMain, t, Twc);
            } else if (placed) {
                lastProvTwc_ = Twc;                // to measure the jump when it merges
                lastProvT_ = t;
            }
            if (placed) pushGoodFrame(cur, ref, Tcw, Twc, t);
            setProvisional(placed && !inMain);
        } else if (placed && !poseConfirmed(t, Twc)) {
            return;
        }
        if (p_.navFrames && !placed) {
            // Lost and re-started in a temporary map: no pose for Nav2 (the
            // safety gate holds the car) until the map merges back.
            slamLost_ = true;
            if (!bridgeOn())
                RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                    "[nav] SLAM is in a temporary map (tracking was lost): no robot pose for Nav2 "
                    "until it merges back into the map it started in");
        }
        if (placed) {
            publishOdom(Twc, stamp);
            if (p_.navFrames) {
                if (p_.poseSource == "slam") publishSlamBase(Twc, stamp);
                else updateCorrection(Twc, t);
                lastPoseWall_ = std::chrono::steady_clock::now();   // safety gate
                havePose_ = true;
                slamLost_ = false;
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

    // One odometry sample: the base driver's wheel pose (x, y, yaw), and the
    // same path re-integrated with the IMU heading (xi, yi, yawi; yawi is
    // unwrapped). Without IMU both are the same.
    struct Odo { double t, x, y, yaw, xi, yi, yawi; };

    static double yawOf(const geometry_msgs::msg::Quaternion& q)
    {
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    // Motor board IMU (jetracer: yaw in 0.1 deg steps, int16 on the wire, so
    // it wraps after ~18 turns: a step over ~30 deg between two samples is
    // such a wrap or a glitch, and is skipped).
    void onImu(const sensor_msgs::msg::Imu& m)
    {
        const double yaw = yawOf(m.orientation);
        std::lock_guard<std::mutex> lk(nav_mx_);
        if (haveImu_) {
            const double d = std::remainder(yaw - imuRaw_, 2.0 * M_PI);
            if (std::fabs(d) < 0.5) imuYaw_ += d;
            else ++imuGlitches_;
        }
        imuRaw_ = yaw;
        imuT_ = rclcpp::Time(m.header.stamp).seconds();
        haveImu_ = true;
    }

    void onWheelOdom(const nav_msgs::msg::Odometry& m)
    {
        const double t = rclcpp::Time(m.header.stamp).seconds();
        const double x = m.pose.pose.position.x, y = m.pose.pose.position.y;
        const double yaw = yawOf(m.pose.pose.orientation);
        std::lock_guard<std::mutex> lk(nav_mx_);
        Odo o{t, x, y, yaw, x, y, yaw};
        const bool imuFresh = haveImu_ && std::fabs(imuT_ - t) < 0.2;
        if (!odo_.empty()) {
            const Odo& p = odo_.back();
            if (t <= p.t) return;                      // repeated or out of order
            const double dx = x - p.x, dy = y - p.y;
            const double dyaw = std::remainder(yaw - p.yaw, 2.0 * M_PI);
            if (std::hypot(dx, dy) > 0.5 || std::fabs(dyaw) > 0.5 || t - p.t > 1.0) {
                // A jump or a gap: the driver reconnected, the board restarted
                // or its int16 counters wrapped. Motion across it is unknown,
                // so no bridge may span it (bridgeFrame compares odoBreaks_).
                ++odoBreaks_;
                RCLCPP_WARN(node_->get_logger(),
                    "[grid] wheel odometry jumped (%.2f m, %.0f deg in %.2f s): no provisional map "
                    "can be placed across this", std::hypot(dx, dy), dyaw * 180.0 / M_PI, t - p.t);
                o.xi = p.xi; o.yi = p.yi; o.yawi = p.yawi;
            } else {
                // The board's own step in its body frame, then turned by the
                // chosen heading (IMU when fresh at both ends, else wheels).
                const double h = p.yaw + 0.5 * dyaw;
                const double bx =  std::cos(h) * dx + std::sin(h) * dy;
                const double by = -std::sin(h) * dx + std::cos(h) * dy;
                const double dh = (imuFresh && imuAtLastOdo_) ? imuYaw_ - imuYawLastOdo_ : dyaw;
                const double hi = p.yawi + 0.5 * dh;
                o.xi = p.xi + std::cos(hi) * bx - std::sin(hi) * by;
                o.yi = p.yi + std::sin(hi) * bx + std::cos(hi) * by;
                o.yawi = p.yawi + dh;
            }
        }
        imuAtLastOdo_ = imuFresh;
        imuYawLastOdo_ = imuYaw_;
        odo_.push_back(o);
        // A minute of odometry: the anchor before a loss and the start of the
        // new map are read soon after they happen.
        while (odo_.size() > 2 && (t - odo_.front().t > 60.0 || odo_.size() > 20000)) odo_.pop_front();
    }

    // Odometry at time t (interpolated; the newest sample if t is up to
    // maxAhead s past it). False if there is no odometry around t.
    bool odoAt(double t, Odo& out, double maxAhead = 0.2)
    {
        std::lock_guard<std::mutex> lk(nav_mx_);
        if (odo_.empty() || t < odo_.front().t) return false;
        if (t >= odo_.back().t) {
            if (t - odo_.back().t > maxAhead) return false;
            out = odo_.back();
            out.t = t;
            return true;
        }
        const auto it = std::lower_bound(odo_.begin(), odo_.end(), t,
                                         [](const Odo& o, double v) { return o.t < v; });
        const Odo& b = *it;
        const Odo& a = *(it == odo_.begin() ? it : it - 1);
        const double f = (b.t > a.t) ? (t - a.t) / (b.t - a.t) : 0.0;
        out = {t, a.x + f * (b.x - a.x), a.y + f * (b.y - a.y),
               a.yaw + f * std::remainder(b.yaw - a.yaw, 2.0 * M_PI),
               a.xi + f * (b.xi - a.xi), a.yi + f * (b.yi - a.yi), a.yawi + f * (b.yawi - a.yawi)};
        return true;
    }

    // Wheel pose at time t, as the base driver reports it (pose source wheels).
    bool wheelAt(double t, Pose2D& out)
    {
        Odo o;
        if (!odoAt(t, o)) return false;
        out = {t, o.x, o.y, o.yaw};
        return true;
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

    // Pass Nav2's command on only while the SLAM pose is fresh and the base
    // driver is alive. A pose older than 0.3 s stops the car at once (a few
    // zero commands, then silence: the base driver stops on its own after 1 s
    // without commands, and the keyboard can drive the car back). The Nav2
    // goal is cancelled only on a real loss -- tracking failed, or SLAM is in
    // a temporary map -- or once the pose is 1.5 s old: a busy tracking thread
    // delays the pose by fractions of a second, and cancelling on every such
    // delay made the planner send a new goal each time. A base driver that
    // stopped reporting (motor board unplugged, or the driver died) cancels at once.
    void gateCommand(const geometry_msgs::msg::Twist& m)
    {
        const auto now = std::chrono::steady_clock::now();
        const double age = havePose_.load()
            ? std::chrono::duration<double>(now - lastPoseWall_.load()).count()
            : 1e9;
        const bool poseStale = p_.poseSource == "slam" && age > 0.3;
        const bool driverNever = p_.expectDriver && !driverSeen_.load() &&
            now - driverStart_ > std::chrono::seconds(5);
        const bool driverDead = driverNever || (driverSeen_.load() &&
            (!driverOk_.load() || now - driverWall_.load() > std::chrono::milliseconds(500)));
        if (poseStale || driverDead) {
            const bool lost = slamLost_.load();
            const bool cancel = driverDead || lost || age > 1.5;
            const bool moving = m.linear.x != 0.0 || m.angular.z != 0.0;
            if (!gateHolding_) {
                if (driverNever)
                    RCLCPP_ERROR(node_->get_logger(),
                        "[nav] NO BASE DRIVER: no heartbeat from the jetracer driver since the start "
                        "(it is not running): holding the car still and cancelling the Nav2 goal");
                else if (driverDead)
                    RCLCPP_WARN(node_->get_logger(),
                        "[nav] base driver not responding (motor board unplugged, or the driver "
                        "died): holding the car still and cancelling the Nav2 goal");
                else
                    RCLCPP_WARN(node_->get_logger(),
                        "[nav] no SLAM pose for %.1f s: holding the car still",
                        age > 1e8 ? -1.0 : age);
                gateStops_ = 5;
                gateHolding_ = true;
                gateCancelled_ = false;
            }
            if (cancel && (!gateCancelled_ || (moving && now - lastCancel_ > std::chrono::seconds(2)))) {
                if (!gateCancelled_ && !driverDead)
                    RCLCPP_WARN(node_->get_logger(),
                        "[nav] %s: cancelling the Nav2 goal; drive back with the keyboard until "
                        "tracking returns, then give the goal again",
                        lost ? "tracking lost (or not in the main map)" : "no SLAM pose for 1.5 s");
                gateCancelled_ = true;
                lastCancel_ = now;
                for (auto& c : cancel_)   // an empty request cancels all goals
                    if (c->service_is_ready())
                        c->async_send_request(std::make_shared<action_msgs::srv::CancelGoal::Request>());
            }
            if (gateStops_ > 0) {
                --gateStops_;
                cmd_out_->publish(geometry_msgs::msg::Twist());
            }
            return;
        }
        if (gateHolding_)
            RCLCPP_INFO(node_->get_logger(), "[nav] %s: passing Nav2 commands again",
                        gateCancelled_ ? "SLAM pose and base driver are back (the goal was cancelled)"
                                       : "SLAM pose is back in time (the goal was kept)");
        gateHolding_ = false;
        cmd_out_->publish(m);
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

    // ── provisional maps (top of file) ────────────────────────────────────
    // All of this runs on the tracking thread; render() reads placedT_ only.
    bool bridgeOn() const { return p_.navFrames && p_.bridge; }

    // A keyframe's current pose (Tcw); a culled keyframe is followed up its
    // parents, as ORB-SLAM3 does when it saves trajectories. Null if lost.
    static ORB_SLAM3::KeyFrame* chase(ORB_SLAM3::KeyFrame* k, Sophus::SE3f& Tkw)
    {
        Sophus::SE3f Trw;
        for (int guard = 0; k && k->isBad() && guard < 1000; ++guard) {
            Trw = Trw * k->mTcp;
            k = k->GetParent();
        }
        if (!k || k->isBad()) return nullptr;
        Tkw = Trw * k->GetPose();
        return k;
    }

    // The base pose to use for the gap: the IMU-heading track or the wheels'
    // own (alt = the other one than grid.bridge_heading).
    Pose2D pick(const Odo& o, bool alt) const
    {
        const bool imu = (p_.bridgeHeading == "imu") != alt;
        return imu ? Pose2D{o.t, o.xi, o.yi, o.yawi} : Pose2D{o.t, o.x, o.y, o.yaw};
    }

    static Sophus::SE3f planar(const Pose2D& p)
    {
        return Sophus::SE3f(Eigen::Quaternionf(Eigen::AngleAxisf((float)p.yaw, Eigen::Vector3f::UnitZ())),
                            Eigen::Vector3f((float)p.x, (float)p.y, 0.f));
    }

    // Camera pose as the planar base pose in map_nav: x, y, yaw.
    Eigen::Vector3d basePose(const Sophus::SE3f& Twc) const
    {
        const Sophus::SE3f Tnb = T_base_cam_ * Twc * T_cam_base_;
        const Eigen::Matrix3f R = Tnb.rotationMatrix();
        return {Tnb.translation().x(), Tnb.translation().y(), std::atan2(R(1, 0), R(0, 0))};
    }

    struct GoodFrame {
        double t;
        ORB_SLAM3::KeyFrame* kf;     // its reference keyframe
        Sophus::SE3f Tcr;            // Tcw(frame) = Tcr * Tcw(kf)
        Sophus::SE3f TwcMain;        // the camera in the main map, as published
        int nTracked;
        ORB_SLAM3::Map* map;
    };
    // The last well-tracked frame before a loss.
    struct Anchor {
        ORB_SLAM3::KeyFrame* kf = nullptr;
        Sophus::SE3f Tcr, TwcMain;   // TwcMain: fallback if the keyframe's map is reset later
        double t = 0.0;
        int nTracked = 0;
        Odo w{};
        bool odoOk = false;
        long breaks = 0;
    };
    // A map other than the main one, and where it sits in the main map.
    struct Place {
        ORB_SLAM3::KeyFrame* init = nullptr;  // its first keyframe = its origin
        Sophus::SE3f TcwInit;                 // that keyframe's pose in it (fixed by bundle adjustment)
        Odo wb{};                             // odometry when it started
        Anchor a;
        bool valid = false, accepted = false;
        std::string why;
        double okSince = 0.0, lastT = 0.0, acceptedT = 0.0;
        Sophus::SE3f T, Talt;                 // main <- this map (chosen heading; the other one)
        bool haveT = false, haveTalt = false;
    };

    void pushGoodFrame(ORB_SLAM3::Map* cur, ORB_SLAM3::KeyFrame* ref, const Sophus::SE3f& Tcw,
                       const Sophus::SE3f& TwcMain, double t)
    {
        int n = 0;
        for (ORB_SLAM3::MapPoint* mp : slam_->GetTrackedMapPoints())
            if (mp) ++n;
        good_.push_back({t, ref, Tcw * ref->GetPose().inverse(), TwcMain, n, cur});
        while (!good_.empty() && t - good_.front().t > 3.0) good_.pop_front();
        lastPlacedMap_ = cur;
    }

    // Tracking just failed (or SLAM moved to a new map): keep the newest
    // recent frame that tracked well as the anchor of the gap.
    void freezeAnchor()
    {
        if (haveAnchor_ || good_.empty()) return;
        const GoodFrame* g = &good_.back();
        for (auto it = good_.rbegin(); it != good_.rend(); ++it)
            if (it->nTracked >= p_.anchorMinPoints) { g = &*it; break; }
        Anchor a;
        a.kf = g->kf; a.Tcr = g->Tcr; a.TwcMain = g->TwcMain;
        a.t = g->t; a.nTracked = g->nTracked;
        a.breaks = odoBreaks_.load();
        a.odoOk = odoAt(g->t, a.w);
        anchor_ = a;
        haveAnchor_ = true;
        RCLCPP_INFO(node_->get_logger(),
            "[nav] tracking lost: anchored at the frame %.2f s before the last good one (%d tracked "
            "points)%s", good_.back().t - g->t, g->nTracked,
            a.odoOk ? "; a new SLAM map will be placed relative to it by odometry"
                    : "; no wheel odometry at that time, so a new map cannot be placed");
        good_.clear();
    }

    // main <- X for the map X (recursively through the maps its anchor was in).
    bool placeOf(ORB_SLAM3::Map* X, ORB_SLAM3::Map* pMain, bool alt, Sophus::SE3f& out, int depth,
                 Sophus::SE3f* TwaOut = nullptr)
    {
        const auto it = places_.find(X);
        if (it == places_.end()) return false;
        const Place& r = it->second;
        if (!r.valid || (depth > 0 && !r.accepted)) return false;
        Sophus::SE3f Twa;              // the anchor camera in the main map
        bool ok = false;
        Sophus::SE3f Tkw;
        if (ORB_SLAM3::KeyFrame* k = chase(r.a.kf, Tkw)) {
            ORB_SLAM3::Map* P = k->GetMap();
            const Sophus::SE3f TwaP = (r.a.Tcr * Tkw).inverse();
            Sophus::SE3f TmP;
            if (P == pMain) { Twa = TwaP; ok = true; }
            else if (P && P != X && depth < 8 && placeOf(P, pMain, alt, TmP, depth + 1)) {
                Twa = TmP * TwaP;
                ok = true;
            }
        }
        if (!ok) Twa = r.a.TwcMain;    // its map was reset: where it was when the loss happened
        const Sophus::SE3f Tab = T_cam_base_ * planar(pick(r.a.w, alt)).inverse() *
                                 planar(pick(r.wb, alt)) * T_base_cam_;
        out = Twa * Tab * r.TcwInit;
        if (TwaOut) *TwaOut = Twa;
        return true;
    }

    // A tracked frame in a map other than the main one. True (and Tmx =
    // main <- that map) once the map is placed and usable.
    bool bridgeFrame(ORB_SLAM3::Map* cur, ORB_SLAM3::Map* pMain, double t,
                     Sophus::SE3f& Tmx, std::string& why)
    {
        if (lastPlacedMap_ && cur != lastPlacedMap_) freezeAnchor();   // new map without a failed frame
        ORB_SLAM3::KeyFrame* init = cur->GetOriginKF();
        Place& r = places_[cur];
        if (r.init != init || (!r.init && r.why.empty())) {
            // A new map, or this Map object reset and started again.
            r = Place{};
            r.init = init;
            r.okSince = r.lastT = t;
            if (!init) r.why = "the new map has no keyframe yet";
            else if (!haveAnchor_) r.why = "no well-tracked frame before the loss to anchor it to";
            else if (!anchor_.odoOk) r.why = "no wheel odometry at the last good frame before the loss";
            else if (!odoAt(init->mTimeStamp, r.wb, 0.5)) r.why = "no wheel odometry when the new map started";
            else {
                r.TcwInit = init->GetPose();
                r.a = anchor_;
                r.valid = true;
            }
        }
        if (t - r.lastT > 0.5) r.okSince = t;      // tracking broke inside this map: start again
        r.lastT = t;
        if (!r.valid) { why = r.why; return false; }
        if (!r.accepted) {
            if (odoBreaks_.load() != r.a.breaks) {
                r.valid = false;
                r.why = "the wheel odometry jumped or stopped during the gap (motor board reconnected?)";
                why = r.why;
                return false;
            }
            const long pts = (long)cur->MapPointsInMap();
            if (pts < p_.bridgeMinPoints || t - r.okSince < p_.bridgeMinTime) {
                char b[160];
                std::snprintf(b, sizeof b, "the new map is not usable yet: %ld map points (needs %d), "
                              "tracked %.1f s (needs %.1f)", pts, p_.bridgeMinPoints, t - r.okSince,
                              p_.bridgeMinTime);
                why = b;
                return false;
            }
        }
        Sophus::SE3f T, Twa;
        if (!placeOf(cur, pMain, false, T, 0, &Twa)) { why = "it cannot be placed"; return false; }
        r.T = T;
        r.haveT = true;
        r.haveTalt = placeOf(cur, pMain, true, r.Talt, 0);
        if (!r.accepted) {
            r.accepted = true;
            r.acceptedT = t;
            const Pose2D wa = pick(r.a.w, false), wb = pick(r.wb, false);
            const Eigen::Vector3d o = basePose(T * r.TcwInit.inverse());
            RCLCPP_WARN(node_->get_logger(),
                "[nav] tracking continues in a new SLAM map (%ld map points), placed in the main map "
                "by %s-heading odometry across a %.1f s gap (%.2f m and %.0f deg driven; its origin at "
                "x %.2f, y %.2f, yaw %.0f deg): provisional until ORB-SLAM3 merges it into the main map",
                (long)cur->MapPointsInMap(), p_.bridgeHeading.c_str(), wb.t - wa.t,
                std::hypot(wb.x - wa.x, wb.y - wa.y), (wb.yaw - wa.yaw) * 180.0 / M_PI,
                o.x(), o.y(), o.z() * 180.0 / M_PI);
            const Eigen::Vector3d an = basePose(Twa);
            RCLCPP_INFO(node_->get_logger(),
                "[nav] bridge detail: anchor t %.3f at x %.3f y %.3f yaw %.2f deg (map_nav), odometry there "
                "x %.3f y %.3f yaw %.2f deg (t %.3f); origin t %.3f, odometry x %.3f y %.3f yaw %.2f deg (t %.3f)",
                r.a.t, an.x(), an.y(), an.z() * 180.0 / M_PI, wa.x, wa.y, wa.yaw * 180.0 / M_PI, wa.t,
                r.init->mTimeStamp, wb.x, wb.y, wb.yaw * 180.0 / M_PI, wb.t);
        }
        haveAnchor_ = false;           // this map keeps its own copy; a loss in it anchors in it
        updatePlacedT(pMain);
        Tmx = T;
        return true;
    }

    // For render(): main <- X of every placed map (scans in them are drawn).
    void updatePlacedT(ORB_SLAM3::Map* pMain)
    {
        std::map<ORB_SLAM3::Map*, Sophus::SE3f> m;
        for (auto& [X, r] : places_) {
            if (!r.accepted) continue;
            Sophus::SE3f T;
            if (placeOf(X, pMain, false, T, 0)) m[X] = T;
        }
        std::lock_guard<std::mutex> lk(bridge_mx_);
        placedT_.swap(m);
    }

    // A pose far from the last published one (0.3 m or 15 deg) is used only once the
    // next frame agrees with it (within 0.1 m and 5 deg). At a merge ORB-SLAM3 can move
    // the reference keyframe into the main map while the frame's pose is still in the
    // merged map's coordinates: on the real robot (21 November) one frame came out
    // 5.4 m and 86 deg off (the merged map's placement). A real jump (a merge's
    // correction, a loop closure) holds for the next frames too and costs one frame.
    bool poseConfirmed(double t, const Sophus::SE3f& Twc)
    {
        auto near = [](const Sophus::SE3f& a, const Sophus::SE3f& b, float m, float deg) {
            return (a.translation() - b.translation()).norm() < m &&
                   (a.so3().inverse() * b.so3()).log().norm() < deg * float(M_PI) / 180.0f;
        };
        const bool ok = !havePub_ || near(Twc, pubTwc_, 0.3f, 15.0f) ||
                        (haveCand_ && t - candT_ < 0.5 && near(Twc, candTwc_, 0.1f, 5.0f)) ||
                        t - pubT_ > 1.0;              // frames that never agree: do not stall
        if (!ok) {
            if (!haveCand_) RCLCPP_INFO(node_->get_logger(),
                "[nav] the SLAM pose jumped %.2f m: waiting for the next frame to confirm it",
                (Twc.translation() - pubTwc_.translation()).norm());
            candTwc_ = Twc; candT_ = t; haveCand_ = true;
            return false;
        }
        pubTwc_ = Twc; pubT_ = t; havePub_ = true; haveCand_ = false;
        return true;
    }

    // Tracking in the main map again: report how far off the placement of
    // each provisional map that has just merged was, and forget them.
    void bridgeBackInMain(ORB_SLAM3::Map* pMain, double t, const Sophus::SE3f& Twc)
    {
        if (lastProvT_ >= 0.0) {
            // Back in the main map right after a provisional pose: how far the robot's pose
            // jumped (the new map's drift since it was placed, plus the placement error).
            const Eigen::Vector3d a = basePose(lastProvTwc_), b = basePose(Twc);
            RCLCPP_WARN(node_->get_logger(),
                "[nav] provisional map merged into the main map: the robot pose jumped %.2f m and %.1f deg "
                "(%.2f s after the last provisional pose)", std::hypot(b.x() - a.x(), b.y() - a.y()),
                std::fabs(std::remainder(b.z() - a.z(), 2.0 * M_PI)) * 180.0 / M_PI, t - lastProvT_);
            lastProvT_ = -1.0;
            mergeT_ = t;
        }
        if (places_.empty() && !haveAnchor_) return;
        // ORB-SLAM3 moves a merged map's keyframes into the main map one by one: an origin
        // keyframe not there yet is checked again on the next frames (up to 10 s).
        bool pending = false;
        for (auto it = places_.begin(); it != places_.end();) {
            Place& r = it->second;
            if (!r.haveT) { it = places_.erase(it); continue; }
            Sophus::SE3f Tkw;
            ORB_SLAM3::KeyFrame* k = chase(r.init, Tkw);
            if (!k || k->GetMap() != pMain) {
                if (t - mergeT_ < 10.0) { pending = true; ++it; } else it = places_.erase(it);
                continue;
            }
            const Eigen::Vector3d truth = basePose(Tkw.inverse());   // its origin, now in the main map
            auto off = [&](const Sophus::SE3f& T, double& d, double& a) {
                const Eigen::Vector3d p = basePose(T * r.TcwInit.inverse());
                d = std::hypot(p.x() - truth.x(), p.y() - truth.y());
                a = std::fabs(std::remainder(p.z() - truth.z(), 2.0 * M_PI)) * 180.0 / M_PI;
            };
            double d = 0, a = 0, d2 = -1, a2 = -1;
            off(r.T, d, a);
            {
                const Eigen::Vector3d pl = basePose(r.T * r.TcwInit.inverse());
                Sophus::SE3f Tkw2;
                ORB_SLAM3::KeyFrame* ka = chase(r.a.kf, Tkw2);
                const Eigen::Vector3d an = ka ? basePose((r.a.Tcr * Tkw2).inverse()) : Eigen::Vector3d(NAN, NAN, NAN);
                RCLCPP_INFO(node_->get_logger(),
                    "[nav] merge detail: origin t %.3f placed x %.3f y %.3f yaw %.2f, merged x %.3f y %.3f yaw %.2f; "
                    "anchor t %.3f now x %.3f y %.3f yaw %.2f (map_nav, deg)", r.init->mTimeStamp,
                    pl.x(), pl.y(), pl.z() * 180.0 / M_PI, truth.x(), truth.y(), truth.z() * 180.0 / M_PI,
                    r.a.t, an.x(), an.y(), an.z() * 180.0 / M_PI);
            }
            if (r.haveTalt) off(r.Talt, d2, a2);
            RCLCPP_WARN(node_->get_logger(),
                "[nav] the merge (%.0f s after the placement) moved the new map's origin by %.2f m and %.1f deg "
                "from where odometry placed it: the placement error plus the new map's own drift between its "
                "origin and where it merged (%s heading; %s heading: %.2f m, %.1f deg)", t - r.acceptedT, d, a,
                p_.bridgeHeading.c_str(), p_.bridgeHeading == "imu" ? "wheels" : "imu", d2, a2);
            it = places_.erase(it);
        }
        haveAnchor_ = false;
        if (pending) return;
        std::lock_guard<std::mutex> lk(bridge_mx_);
        placedT_.clear();
    }

    void setProvisional(bool v)
    {
        if (!prov_pub_ || v == provisional_) return;
        provisional_ = v;
        std_msgs::msg::Bool b;
        b.data = v;
        prov_pub_->publish(b);
    }

    // The map the robot navigates and maps in: SLAM's main map once it has
    // qualified (20 keyframes), before that the first map of this run, so the
    // pose and the grid exist from the first frames instead of only after the
    // robot has driven enough for the main map. Normally the first map is the
    // one that qualifies, so nothing jumps; if it is abandoned first, the
    // pose pauses until the main map exists.
    ORB_SLAM3::Map* navMap()
    {
        ORB_SLAM3::Map* pMain = slam_->GetMainMap();
        if (pMain) return pMain;
        ORB_SLAM3::Map* pFirst = firstMap_.load();
        return (pFirst && !pFirst->IsBad()) ? pFirst : nullptr;
    }

    void render()
    {
        ORB_SLAM3::Map* pMain = navMap();
        if (!pMain) return;
        std::vector<std::shared_ptr<const Scan>> scans;
        {
            std::lock_guard<std::mutex> lk(scans_mx_);
            scans = scans_;
        }
        if (scans.empty()) return;
        std::map<ORB_SLAM3::Map*, Sophus::SE3f> placedT;      // provisional maps: main <- map
        {
            std::lock_guard<std::mutex> lk(bridge_mx_);
            placedT = placedT_;
        }

        // Each scan's base pose in map_nav, from its keyframe's CURRENT pose.
        // A culled keyframe is followed up its parents, as ORB-SLAM3 does
        // when it saves trajectories. Scans of a provisional map go through
        // its placement and into the second layer (prov).
        struct Pose2 { float x, y, yaw; const Scan* s; bool prov; };
        std::vector<Pose2> poses;
        poses.reserve(scans.size());
        for (const auto& s : scans) {
            Sophus::SE3f Tkw;
            ORB_SLAM3::KeyFrame* k = chase(s->anchor, Tkw);
            if (!k) continue;
            ORB_SLAM3::Map* m = k->GetMap();
            Sophus::SE3f Twc = (s->Tcr * Tkw).inverse();
            bool prov = false;
            if (m != pMain) {
                const auto it = placedT.find(m);
                if (it == placedT.end()) continue;               // not in the main map (yet)
                Twc = it->second * Twc;
                prov = true;
            }
            const Sophus::SE3f Tnb = T_base_cam_ * Twc * T_cam_sensor_;   // camera_floor in map_nav
            const Eigen::Matrix3f R = Tnb.rotationMatrix();
            poses.push_back({Tnb.translation().x(), Tnb.translation().y(),
                             std::atan2(R(1, 0), R(0, 0)), s.get(), prov});
        }
        if (poses.empty()) return;

        const float res  = (float)p_.resolution;
        // Nav2's global costmap takes the map's size, and its planner cannot
        // aim outside it: without a margin a goal just past where the camera
        // looked (or the car's own outline there) was rejected as unreachable.
        const float pad  = (float)(p_.maxRange + p_.mapMargin) + 2.f * res;
        float minx = 1e9f, miny = 1e9f, maxx = -1e9f, maxy = -1e9f;
        for (const auto& p : poses) {
            minx = std::min(minx, p.x); maxx = std::max(maxx, p.x);
            miny = std::min(miny, p.y); maxy = std::max(maxy, p.y);
        }
        minx -= pad; miny -= pad; maxx += pad; maxy += pad;
        const float hs = (float)p_.mapHalfSize;           // fixed minimum extent around the start
        minx = std::min(minx, -hs); miny = std::min(miny, -hs);
        maxx = std::max(maxx, hs);  maxy = std::max(maxy, hs);
        const int W = (int)std::ceil((maxx - minx) / res);
        const int H = (int)std::ceil((maxy - miny) / res);
        if (W <= 0 || H <= 0 || (long)W * H > 25000000L) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                                 "[grid] map would be %d x %d cells; not rendered", W, H);
            return;
        }

        const float lHit = 0.85f, lMiss = -0.4f, lMin = -2.f, lMax = 3.5f;
        // Two layers: the main map's scans, and the provisional maps' scans,
        // which only fill cells the main map has not seen (their placement
        // is approximate: they must not move or clear the main map's walls).
        std::vector<float> Lmain((size_t)W * H, 0.f), Lprov;
        std::vector<uint8_t> seenMain((size_t)W * H, 0), seenProv;
        const bool anyProv = std::any_of(poses.begin(), poses.end(), [](const Pose2& p) { return p.prov; });
        if (anyProv) { Lprov.assign((size_t)W * H, 0.f); seenProv.assign((size_t)W * H, 0); }
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
            std::vector<float>& L = ps.prov ? Lprov : Lmain;
            std::vector<uint8_t>& seen = ps.prov ? seenProv : seenMain;
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
                    seen[c] = 1;
                    L[c] = std::max(lMin, L[c] + lMiss);
                }
            }
            for (long c : hits) { seen[c] = 1; L[c] = std::min(lMax, L[c] + lHit); }
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
        for (size_t i = 0; i < Lmain.size(); ++i) {
            const float l = (anyProv && !seenMain[i] && seenProv[i]) ? Lprov[i] : Lmain[i];
            g.data[i] = l >= p_.occThreshold ? 100 : (l <= p_.freeThreshold ? 0 : -1);
        }
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
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_out_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_in_;
    std::vector<rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr> cancel_;
    bool gateHolding_ = false;   // gate state; touched only in gateCommand (executor thread)
    bool gateCancelled_ = false; // the goal was cancelled during this hold
    int gateStops_ = 0;
    std::chrono::steady_clock::time_point lastCancel_{};
    std::atomic<std::chrono::steady_clock::time_point> lastPoseWall_{std::chrono::steady_clock::time_point{}};
    std::atomic<bool> havePose_{false};
    std::atomic<bool> slamLost_{false};      // tracking failed or in a temporary map (until the next pose)
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr driver_sub_;
    std::atomic<bool> driverSeen_{false}, driverOk_{false};
    std::chrono::steady_clock::time_point driverStart_{std::chrono::steady_clock::now()};
    std::atomic<std::chrono::steady_clock::time_point> driverWall_{std::chrono::steady_clock::time_point{}};
    std::atomic<ORB_SLAM3::Map*> firstMap_{nullptr};
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
    rclcpp::TimerBase::SharedPtr corr_timer_;
    std::mutex nav_mx_;                      // guards the odometry and IMU state below and corr_
    std::deque<Odo> odo_;
    std::atomic<long> odoBreaks_{0};         // jumps or gaps in the wheel odometry so far
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    bool haveImu_ = false, imuAtLastOdo_ = false;
    double imuRaw_ = 0.0, imuYaw_ = 0.0, imuT_ = -1e9, imuYawLastOdo_ = 0.0;
    long imuGlitches_ = 0;
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

    // Provisional maps (tracking thread; placedT_ also read by render()).
    std::deque<GoodFrame> good_;             // well-tracked frames of the last 3 s, in placed maps
    Anchor anchor_;
    bool haveAnchor_ = false;
    std::map<ORB_SLAM3::Map*, Place> places_;
    Sophus::SE3f lastProvTwc_;             // the last provisional pose (main map)
    double lastProvT_ = -1.0, mergeT_ = -1e9;
    Sophus::SE3f pubTwc_, candTwc_;        // poseConfirmed: the last used pose, a jump waiting
    double pubT_ = 0.0, candT_ = 0.0;
    bool havePub_ = false, haveCand_ = false;
    ORB_SLAM3::Map* lastPlacedMap_ = nullptr;
    bool provisional_ = false;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr prov_pub_;
    std::mutex bridge_mx_;
    std::map<ORB_SLAM3::Map*, Sophus::SE3f> placedT_;

    std::mutex scans_mx_;
    std::vector<std::shared_ptr<const Scan>> scans_;

    std::atomic<bool> stop_{false};
    std::mutex run_mx_;
    std::condition_variable cv_;
    std::thread worker_;
};

}  // namespace orbslam3_nav
