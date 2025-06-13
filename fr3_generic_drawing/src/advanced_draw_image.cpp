// fr3_sketch_drawer.cpp — Final version
// Full image sketching pipeline with orientation constraint, jitter, adaptive hatching,
// stroke sequencing, pressure-modulated Z, and Cartesian-only planning.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <random>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>
#include <numeric>

// Clamp fallback for C++14
template <typename T>
T clamp(T val, T low, T high) {
    return std::max(low, std::min(val, high));
}

const std::string IMAGE_PATH = "/home/qpaig/my_ros2_ws/src/fr3_generic_drawing/src/star_outline.jpg";
const double CONVERSION_FACTOR = 0.0003;      // px to meters
const double DRAWING_HEIGHT = 0.3; // Table height
const double TIP_OFFSET = 0.015;
const double Z_DOWN_BASE = DRAWING_HEIGHT + TIP_OFFSET;
const double Z_PENCIL_RAISED = Z_DOWN_BASE + 0.02;
const double PRESSURE_RANGE = 0.002;
const double X_ORIGIN = 0.4;
const double Y_ORIGIN = 0.0;

geometry_msgs::msg::Quaternion vertical_orientation() {
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, 0);
    q.normalize();
    return tf2::toMsg(q);
}

double pressure_z(uint8_t intensity) {
    double t = intensity / 255.0;
    return Z_DOWN_BASE - PRESSURE_RANGE + (2 * PRESSURE_RANGE * t);
}

geometry_msgs::msg::Pose image_to_pose(int px, int py, int img_w, int img_h, double z, geometry_msgs::msg::Quaternion ori) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = X_ORIGIN + (img_h/2.0 - py) * CONVERSION_FACTOR;
    pose.position.y = Y_ORIGIN + (img_w/2.0 - px) * CONVERSION_FACTOR;
    pose.position.z = z;
    pose.orientation = ori;
    return pose;
}

std::vector<std::vector<cv::Point>> extract_hatches(const cv::Mat &img, int spacing = 5, int t1 = 200, int t2 = 100) {
    std::vector<std::vector<cv::Point>> strokes;
    int h = img.rows, w = img.cols;
    for (int y = 0; y < h; y += spacing) {
        cv::Point start;
        bool dark = false;
        for (int x = 0; x < w; ++x) {
            uchar i = img.at<uchar>(y, x);
            if (i < t1 && !dark) { start = {x, y}; dark = true; }
            if ((i >= t1 || x == w - 1) && dark) {
                strokes.push_back({start, {x, y}}); dark = false;
            }
        }
    }
    for (int x = 0; x < w; x += spacing) {
        cv::Point start;
        bool dark = false;
        for (int y = 0; y < h; ++y) {
            uchar i = img.at<uchar>(y, x);
            if (i < t2 && !dark) { start = {x, y}; dark = true; }
            if ((i >= t2 || y == h - 1) && dark) {
                strokes.push_back({start, {x, y}}); dark = false;
            }
        }
    }
    return strokes;
}

std::vector<size_t> nearest_path_order(const std::vector<std::vector<cv::Point>> &paths) {
    std::vector<size_t> order, pool(paths.size());
    std::iota(pool.begin(), pool.end(), 0);
    order.push_back(pool.front()); pool.erase(pool.begin());
    cv::Point last = paths[order.back()].back();
    while (!pool.empty()) {
        auto it = std::min_element(pool.begin(), pool.end(), [&](size_t a, size_t b) {
            return norm(last - paths[a].front()) < norm(last - paths[b].front());
        });
        order.push_back(*it);
        last = paths[*it].back();
        pool.erase(it);
    }
    return order;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("fr3_sketch_node");
    moveit::planning_interface::MoveGroupInterface mg(node, "fr3_arm");
    mg.setPlanningTime(10);
    mg.setMaxVelocityScalingFactor(0.15);
    mg.setPoseReferenceFrame("base");

    // Orientation constraint: vertical downward with tolerance
    moveit_msgs::msg::OrientationConstraint oc;
    oc.link_name = mg.getEndEffectorLink();
    oc.header.frame_id = "base";
    oc.orientation = vertical_orientation();
    oc.absolute_x_axis_tolerance = 0.1;
    oc.absolute_y_axis_tolerance = 0.1;
    oc.absolute_z_axis_tolerance = 0.1;
    oc.weight = 1.0;
    moveit_msgs::msg::Constraints pc;
    pc.orientation_constraints.push_back(oc);
    mg.setPathConstraints(pc);

    // Load and process image
    cv::Mat img = cv::imread(IMAGE_PATH, cv::IMREAD_GRAYSCALE);
    cv::Mat blur, edges;
    cv::blur(img, blur, {3, 3});
    cv::Canny(blur, edges, 100, 200);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    // Add sketch jitter
    std::default_random_engine gen;
    std::uniform_int_distribution<int> jitter(-2, 2);
    for (auto& c : contours) {
        for (auto& p : c) {
            p.x = clamp(p.x + jitter(gen), 0, img.cols - 1);
            p.y = clamp(p.y + jitter(gen), 0, img.rows - 1);
        }
    }

    auto hatches = extract_hatches(img);
    std::vector<std::vector<cv::Point>> all_strokes = contours;
    all_strokes.insert(all_strokes.end(), hatches.begin(), hatches.end());

    auto order = nearest_path_order(all_strokes);
    for (size_t i : order) {
        const auto& stroke = all_strokes[i];
        if (stroke.empty()) continue;
        std::vector<geometry_msgs::msg::Pose> wp;
        geometry_msgs::msg::Quaternion ori = vertical_orientation();
        // pen-up to start
        int sx = stroke.front().x, sy = stroke.front().y;
        wp.push_back(image_to_pose(sx, sy, img.cols, img.rows, Z_PENCIL_RAISED, ori));
        // pen-down to draw
        for (const auto& pt : stroke) {
            int x = clamp(pt.x, 0, img.cols - 1);
            int y = clamp(pt.y, 0, img.rows - 1);
            double z = pressure_z(img.at<uchar>(y, x));
            wp.push_back(image_to_pose(x, y, img.cols, img.rows, z, ori));
        }
        // lift pen
        int ex = stroke.back().x, ey = stroke.back().y;
        wp.push_back(image_to_pose(ex, ey, img.cols, img.rows, Z_PENCIL_RAISED, ori));

        moveit_msgs::msg::RobotTrajectory traj;
        mg.setStartStateToCurrentState();
        mg.computeCartesianPath(wp, 0.005, 0.0, traj);
        moveit::planning_interface::MoveGroupInterface::Plan p;
        p.trajectory_ = traj;
        mg.execute(p);
    }

    rclcpp::shutdown();
    return 0;
}