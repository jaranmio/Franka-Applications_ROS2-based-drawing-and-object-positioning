// Segment-fragmented drawing for robustness
// Breaks strokes into small fragments and skips segments with jumps

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
#include <stack>
#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#define NANOSVG_IMPLEMENTATION
#include "nanosvg.h"

const std::string IMAGE_PATH = "/home/qpaig/my_ros2_ws/src/fr3_generic_drawing/src/star.svg";
const double CONVERSION_FACTOR = 0.0001257 * 1.0;
const double DRAWING_HEIGHT = 0.201;
// 0.190 Conte a Paris Sanguine maybe add 1cm on measured length
// 0.132 - creata color monolith HB
// 0.16 (approx. not working) - carbon sketch
const double Z_PENCIL_DOWN = DRAWING_HEIGHT;
const double RAISING_AMOUNT = 0.05;
const double Z_PENCIL_RAISED = Z_PENCIL_DOWN + RAISING_AMOUNT;
const double X_ORIGIN = 0.49;
const double Y_ORIGIN = 0.0;
const int SEGMENT_SIZE = 10;

struct Point
{
    float x, y;
};

inline bool is_far_enough(const Point &a, const Point &b, float min_dist_sq = 1.0f)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return dx * dx + dy * dy >= min_dist_sq;
}

void flattenCubicBezier(
    float x1, float y1, float x2, float y2,
    float x3, float y3, float x4, float y4,
    float tol, std::vector<Point> &out, int level = 0)
{
    if (level > 10)
    {
        Point p = {x4, y4};
        if (out.empty() || is_far_enough(p, out.back()))
            out.push_back(p);
        return;
    }

    float x12 = (x1 + x2) * 0.5f;
    float y12 = (y1 + y2) * 0.5f;
    float x23 = (x2 + x3) * 0.5f;
    float y23 = (y2 + y3) * 0.5f;
    float x34 = (x3 + x4) * 0.5f;
    float y34 = (y3 + y4) * 0.5f;

    float x123 = (x12 + x23) * 0.5f;
    float y123 = (y12 + y23) * 0.5f;
    float x234 = (x23 + x34) * 0.5f;
    float y234 = (y23 + y34) * 0.5f;

    float x1234 = (x123 + x234) * 0.5f;
    float y1234 = (y123 + y234) * 0.5f;

    float dx = x4 - x1;
    float dy = y4 - y1;
    float d2 = std::abs((x2 - x4) * dy - (y2 - y4) * dx);
    float d3 = std::abs((x3 - x4) * dy - (y3 - y4) * dx);
    if ((d2 + d3) * (d2 + d3) < tol * (dx * dx + dy * dy))
    {
        Point p = {x4, y4};
        if (out.empty() || is_far_enough(p, out.back()))
            out.push_back(p);
    }
    else
    {
        flattenCubicBezier(x1, y1, x12, y12, x123, y123, x1234, y1234, tol, out, level + 1);
        flattenCubicBezier(x1234, y1234, x234, y234, x34, y34, x4, y4, tol, out, level + 1);
    }
}

std::vector<std::vector<Point>> extract_svg_paths(NSVGimage *image, float tol = 1.5f)
{
    std::vector<std::vector<Point>> all_paths;
    for (NSVGshape *shape = image->shapes; shape; shape = shape->next)
    {
        for (NSVGpath *path = shape->paths; path; path = path->next)
        {
            std::vector<Point> polyline;
            float *pts = path->pts;
            int npts = path->npts;
            if (npts < 1)
                continue;

            Point start = {pts[0], pts[1]};
            polyline.push_back(start);

            for (int i = 0; i < npts - 1; i += 3)
            {
                float *p = &pts[i * 2];
                flattenCubicBezier(
                    p[0], p[1], p[2], p[3],
                    p[4], p[5], p[6], p[7],
                    tol, polyline);
            }

            if (polyline.size() >= 2)
            {
                all_paths.push_back(std::move(polyline));
            }
        }
    }
    nsvgDelete(image);
    return all_paths;
}

template <typename T>
T clamp(T val, T low, T high)
{
    return std::max(low, std::min(val, high));
}

auto vertical_orientation()
{
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, 0);
    q.normalize();
    return tf2::toMsg(q);
}

auto image_to_pose(int px, int py, int img_w, int img_h, double z)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = X_ORIGIN + (img_h / 2.0 - py) * CONVERSION_FACTOR;
    pose.position.y = Y_ORIGIN + (img_w / 2.0 - px) * CONVERSION_FACTOR;
    pose.position.z = z;
    pose.orientation = vertical_orientation();
    return pose;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("fr3_sketch_node", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();

    moveit::planning_interface::MoveGroupInterface mg(node, "fr3_arm");
    mg.startStateMonitor();
    mg.setPlanningTime(10);
    mg.setMaxVelocityScalingFactor(0.3);
    mg.setPoseReferenceFrame("fr3_link0");
    mg.setEndEffectorLink("fr3_hand_tcp");

    moveit_msgs::msg::OrientationConstraint oc;
    oc.link_name = mg.getEndEffectorLink();
    oc.header.frame_id = "fr3_link0";
    oc.orientation = vertical_orientation();
    oc.absolute_x_axis_tolerance = 0.1;
    oc.absolute_y_axis_tolerance = 0.1;
    oc.absolute_z_axis_tolerance = 0.1;
    oc.weight = 1.0;
    moveit_msgs::msg::Constraints path_constraints;
    path_constraints.orientation_constraints.push_back(oc);
    mg.setPathConstraints(path_constraints);

    NSVGimage *image = nsvgParseFromFile(IMAGE_PATH.c_str(), "px", 96.0f);
    if (!image)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to load SVG file: %s", IMAGE_PATH.c_str());
        rclcpp::shutdown();
        return 1;
    }
    float width = image->width;
    float height = image->height;
    auto contours = extract_svg_paths(image);
    for (const auto &stroke : contours)
    {
        if (stroke.size() < 2)
            continue;

        auto start_lifted = image_to_pose(stroke.front().x, stroke.front().y, width, height, Z_PENCIL_RAISED);
        std::vector<geometry_msgs::msg::Pose> lift_start_path = {mg.getCurrentPose().pose, start_lifted};
        moveit_msgs::msg::RobotTrajectory lift_start_traj;
        if (mg.computeCartesianPath(lift_start_path, 0.01, 0.0, lift_start_traj) >= 0.95)
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = lift_start_traj;
            mg.execute(plan);
        }

        for (size_t i = 0; i < stroke.size(); i += SEGMENT_SIZE)
        {
            size_t end_index = std::min(i + SEGMENT_SIZE, stroke.size());
            std::vector<geometry_msgs::msg::Pose> segment;
            for (size_t j = i; j < end_index; ++j)
            {
                int x = clamp(static_cast<int>(stroke[j].x), 0, static_cast<int>(width) - 1);
                int y = clamp(static_cast<int>(stroke[j].y), 0, static_cast<int>(height) - 1);
                segment.push_back(image_to_pose(x, y, width, height, Z_PENCIL_DOWN));
            }
            if (segment.size() < 2)
                continue;
            moveit_msgs::msg::RobotTrajectory seg_traj;
            if (mg.computeCartesianPath(segment, 0.005, 0.0, seg_traj) >= 0.95)
            {
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                plan.trajectory_ = seg_traj;
                mg.execute(plan);
            }
            else
            {
                mg.setPoseTarget(segment.back());
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                if (mg.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
                    mg.execute(plan);
            }
        }

        auto end_lifted = image_to_pose(stroke.back().x, stroke.back().y, width, height, Z_PENCIL_RAISED);
        std::vector<geometry_msgs::msg::Pose> lift_end_path = {mg.getCurrentPose().pose, end_lifted};
        moveit_msgs::msg::RobotTrajectory lift_end_traj;
        if (mg.computeCartesianPath(lift_end_path, 0.01, 0.0, lift_end_traj) >= 0.95)
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = lift_end_traj;
            mg.execute(plan);
        }
    }
    rclcpp::shutdown();

    return 0;
}
