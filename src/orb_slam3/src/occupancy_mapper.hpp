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
// Frames (names are parameters): <grid.global_frame> (default map_nav) is the
// level floor frame under the first camera pose, a static child of the
// ORB-SLAM3 "map" frame; <grid.base_frame> (default base_link) is the level
// floor frame under the camera, a static child of camera_color_optical_frame.
// Nav2 can use map_nav as its global frame and base_link as the robot frame.
// /<agent>/orb_slam3/odom (nav_msgs/Odometry) carries the same pose for Nav2
// nodes that need an odometry topic (/<agent>/odom is the wheel odometry).
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
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
    std::string globalFrame  = "map_nav";
    std::string baseFrame    = "base_link";
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
        static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*node_);

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
        if (!trackingOk) return;
        ORB_SLAM3::KeyFrame* ref = slam_->GetTrackingReferenceKF();
        if (!ref) return;

        const Sophus::SE3f Twc = Tcw.inverse();
        ORB_SLAM3::Map* pMain = slam_->GetMainMap();
        if (pMain && ref->GetMap() == pMain) publishOdom(Twc, stamp);

        const double t = stamp.seconds();
        if (haveLast_) {
            if (t - lastT_ < p_.scanInterval) return;
            const float moved  = (Twc.translation() - lastTwc_.translation()).norm();
            const float turned = (lastTwc_.so3().inverse() * Twc.so3()).log().norm();
            if (moved < p_.scanMove && turned < p_.scanTurnDeg * M_PI / 180.0) return;
        }
        auto scan = buildScan(depthM);
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
        T_cam_base_ = Sophus::SE3f(q, -h * z);
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
        v.push_back(toTf(T_cam_base_, p_.cameraFrame, p_.baseFrame));
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

    std::unique_ptr<Scan> buildScan(const cv::Mat& depth)
    {
        const int B = p_.bins;
        const float fov  = (float)(p_.fovDeg * M_PI / 180.0);
        const float binW = fov / B;
        const float maxR = (float)p_.maxRange;
        std::vector<std::vector<float>> band(B);
        std::vector<float> freeTo(B, -1.f);
        const Eigen::Matrix3f Rbc = T_base_cam_.rotationMatrix();
        const Eigen::Vector3f tbc = T_base_cam_.translation();
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
            const Sophus::SE3f Tnb = T_base_cam_ * Tcw.inverse() * T_cam_base_;
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
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;

    // Mount (written once by the tracking thread before calibrated_ is set).
    Sophus::SE3f T_cam_base_, T_base_cam_;
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
