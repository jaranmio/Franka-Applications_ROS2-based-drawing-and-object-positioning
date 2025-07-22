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
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#define NANOSVG_IMPLEMENTATION
#include "nanosvg.h"

YAML::Node config = YAML::LoadFile("/home/qpaig/my_ros2_ws/src/fr3_generic_drawing/vector_config/config.yaml");
std::string image_file = config["image_file"].as<std::string>();
double percent_coverage = config["coverage"].as<double>(); // as a percentage
double height_rate = (config["height_rate"].as<double>() * std::pow(10, -2.0)) / 2; // to m per 0.5 hour
double pen_height = config["pen_height"].as<double>() * std::pow(10, -2.0); // to m
double paper_length = config["paper_length"].as<double>() * std::pow(10, -2.0); // along x, converted to m
double paper_width = config["paper_width"].as<double>() * std::pow(10, -2.0); // along y, converted to m
double center_x = config["image_center_x"].as<double>() * std::pow(10, -2.0); // to m
double center_y = config["image_center_y"].as<double>() * std::pow(10, -2.0); // to m

const std::string IMAGE_PATH = "/home/qpaig/my_ros2_ws/src/fr3_generic_drawing/vector_config/images/" + image_file;
double CONVERSION_FACTOR;
double DRAWING_HEIGHT = pen_height;
// 0.190 Conte a Paris Sanguine maybe add 1cm on measured length
// 0.154 PRANG peel off HB
// 0.182c Paris Conte Charcoal add groundplate
// 0.132 - creata color monolith HB
// 0.16 (approx. not working) - carbon sketch
const double Z_PENCIL_DOWN = DRAWING_HEIGHT;
const double RAISING_AMOUNT = 0.05;
const double Z_PENCIL_RAISED = Z_PENCIL_DOWN + RAISING_AMOUNT;
const double X_ORIGIN = center_x;
const double Y_ORIGIN = center_y;
const int SEGMENT_SIZE = 10 * 2;
const double SECS_PER_HALF_HOUR = 30 * 60;

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

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();

    moveit::planning_interface::MoveGroupInterface mg(node, "fr3_arm");
    mg.startStateMonitor();
    mg.setPlanningTime(10);
    mg.setMaxVelocityScalingFactor(0.3);
    mg.setPoseReferenceFrame("fr3_link0");
    mg.setEndEffectorLink("fr3_hand_tcp"); // default is 'fr3_link8' pencil_tip

    moveit_msgs::msg::OrientationConstraint oc;
    oc.link_name = mg.getEndEffectorLink();
    oc.header.frame_id = "fr3_link0";
    oc.orientation = vertical_orientation();
    oc.absolute_x_axis_tolerance = 0.1;
    oc.absolute_y_axis_tolerance = 0.1;
    oc.absolute_z_axis_tolerance = 0.1;
    oc.weight = 1.0;

    const double min_height = pen_height - 0.012;
    moveit_msgs::msg::PositionConstraint posc;
    posc.link_name = mg.getEndEffectorLink();  // e.g., "pencil_tip"
    posc.header.frame_id = "fr3_link0";        // or "world" if you're using that as root
    // Define bounding box volume: large in x/y, only upward in z from min_height
    shape_msgs::msg::SolidPrimitive bound;
    bound.type = shape_msgs::msg::SolidPrimitive::BOX;
    bound.dimensions.resize(3);
    bound.dimensions[0] = 10.0;               // x dimension (wide)
    bound.dimensions[1] = 10.0;               // y dimension (wide)
    bound.dimensions[2] = 10.0;                // z dimension (tall enough to allow motion)
    geometry_msgs::msg::Pose region_pose;
    region_pose.position.x = 0.0;
    region_pose.position.y = 0.0;
    region_pose.position.z = min_height + bound.dimensions[2] / 2.0;  // center of box
    region_pose.orientation.w = 1.0;  // identity rotation
    posc.constraint_region.primitives.push_back(bound);
    posc.constraint_region.primitive_poses.push_back(region_pose);
    posc.weight = 10000.0;

    moveit_msgs::msg::Constraints pc;
    pc.orientation_constraints.push_back(oc);
    pc.position_constraints.push_back(posc);
    mg.setPathConstraints(pc);

    auto stamp_and_execute = [&](moveit_msgs::msg::RobotTrajectory &traj,
                                double v_scale = 0.25,
                                double a_scale = 0.25)
    {
        robot_trajectory::RobotTrajectory rt(mg.getRobotModel(),
                                            mg.getName());
        rt.setRobotTrajectoryMsg(*mg.getCurrentState(), traj);

        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        bool ok = iptp.computeTimeStamps(rt, v_scale, a_scale);
        if (!ok)
        {
            RCLCPP_ERROR(node->get_logger(), "Time-parameterisation failed");
            return;
        }
        rt.getRobotTrajectoryMsg(traj);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = traj;
        mg.execute(plan);
        rclcpp::sleep_for(std::chrono::milliseconds(200));                
    };

    NSVGimage *image = nsvgParseFromFile(IMAGE_PATH.c_str(), "px", 96.0f);
    if (!image)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to load SVG file: %s", IMAGE_PATH.c_str());
        rclcpp::shutdown();
        return 1;
    }
    float width = image->height; // inversion is intentional, just for logic of calculating conversion factor
    float length = image->width;

    // find conversion factor that maximizes area coverage
    const double image_aspect_ratio = length / width;
    
    const double paper_aspect_ratio = paper_length / paper_width;

    double limiting_image_dimension;
    double limiting_paper_dimension;
    
    if ((paper_aspect_ratio > 1 && image_aspect_ratio > 1) || (paper_aspect_ratio < 1 && image_aspect_ratio < 1)) {
        // same aspect ratio
        limiting_paper_dimension = std::min(paper_width, paper_length);
        limiting_image_dimension = std::min(width, length);
    } else{
        limiting_paper_dimension = std::min(paper_width, paper_length);
        limiting_image_dimension = std::max(width, length);
    }

    CONVERSION_FACTOR = limiting_paper_dimension / limiting_image_dimension;
    CONVERSION_FACTOR *= std::sqrt(percent_coverage / 100);
    RCLCPP_INFO(node->get_logger(), ("CONVERSION FACTOR: " + std::to_string(CONVERSION_FACTOR)).c_str());

    auto contours = extract_svg_paths(image);

    auto START = std::chrono::high_resolution_clock::now(); // start clock
    double half_hours_tracking = 0; // variable to make sure we reduce exactly once per half hour

    for (const auto &stroke : contours)
    {
        if (stroke.size() < 2)
            continue;

        auto start_lifted = image_to_pose(stroke.front().x, stroke.front().y, image->width, image->height, Z_PENCIL_RAISED);
        std::vector<geometry_msgs::msg::Pose> lift_start_path = {mg.getCurrentPose().pose, start_lifted};
        moveit_msgs::msg::RobotTrajectory lift_start_traj;
        if (mg.computeCartesianPath(lift_start_path, 0.01, 0.0, lift_start_traj) >= 0.95)
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = lift_start_traj;
            stamp_and_execute(lift_start_traj);
        }

        for (size_t i = 0; i < stroke.size(); i += SEGMENT_SIZE)
        {
            double ELAPSED_TIME_SECS = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - START).count();
            const int HALF_HOURS_ELASPED = std::floor(ELAPSED_TIME_SECS / SECS_PER_HALF_HOUR);
            
            if (HALF_HOURS_ELASPED > half_hours_tracking) { // reduce height only once per half hour
                const double HEIGHT_ADJUSTMENT = HALF_HOURS_ELASPED * height_rate;
                DRAWING_HEIGHT -= HEIGHT_ADJUSTMENT;
                RCLCPP_INFO(node->get_logger(), ("NEW HEIGHT SET: " + std::to_string(DRAWING_HEIGHT) + "m").c_str());
                half_hours_tracking++;
            }

            size_t end_index = std::min(i + SEGMENT_SIZE, stroke.size());
            std::vector<geometry_msgs::msg::Pose> segment;
            for (size_t j = i; j < end_index; ++j)
            {
                int x = clamp(static_cast<int>(stroke[j].x), 0, static_cast<int>(image->width) - 1);
                int y = clamp(static_cast<int>(stroke[j].y), 0, static_cast<int>(image->height) - 1);
                segment.push_back(image_to_pose(x, y, image->width, image->height, Z_PENCIL_DOWN));
            }
            if (segment.size() < 2)
                continue;
            moveit_msgs::msg::RobotTrajectory seg_traj;
            if (mg.computeCartesianPath(segment, 0.005, 0.0, seg_traj) >= 0.95)
            {
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                plan.trajectory_ = seg_traj;
                stamp_and_execute(seg_traj);
            }
            else
            {
                mg.setPoseTarget(segment.back());
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                if (mg.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
                    mg.execute(plan);
                    rclcpp::sleep_for(std::chrono::milliseconds(200));
            }
        }

        auto end_lifted = image_to_pose(stroke.back().x, stroke.back().y, image->width, image->height, Z_PENCIL_RAISED);
        std::vector<geometry_msgs::msg::Pose> lift_end_path = {mg.getCurrentPose().pose, end_lifted};
        moveit_msgs::msg::RobotTrajectory lift_end_traj;
        if (mg.computeCartesianPath(lift_end_path, 0.01, 0.0, lift_end_traj) >= 0.95)
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = lift_end_traj;
            stamp_and_execute(lift_end_traj);
        }
    }

    double ELAPSED_TIME_SECS = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - START).count();

    RCLCPP_INFO(node->get_logger(), ("TOTAL DRAWING TIME: " + std::to_string(ELAPSED_TIME_SECS) + "secs").c_str());
    
    rclcpp::shutdown();

    return 0;
}
