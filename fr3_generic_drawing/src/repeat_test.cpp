// Segment-fragmented drawing for robustness
// Breaks strokes into small fragments and skips segments with jumps

// How to log: RCLCPP_ERROR(node->get_logger(), "Text");

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
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

YAML::Node config = YAML::LoadFile("/home/qpaig/my_ros2_ws/src/fr3_generic_drawing/raster_config/config.yaml");
std::string image_file = config["image_file"].as<std::string>();
double percent_coverage = config["coverage"].as<double>(); // as a percentage
double height_rate = (config["height_rate"].as<double>() * std::pow(10, -2.0)) / 2; // to m per 0.5 hour
double pen_height = config["pen_height"].as<double>() * std::pow(10, -2.0); // to m
double paper_length = config["paper_length"].as<double>() * std::pow(10, -2.0); // along x, converted to m
double paper_width = config["paper_width"].as<double>() * std::pow(10, -2.0); // along y, converted to m
double center_x = config["image_center_x"].as<double>() * std::pow(10, -2.0); // to m
double center_y = config["image_center_y"].as<double>() * std::pow(10, -2.0); // to m

const std::string IMAGE_PATH = "/home/qpaig/my_ros2_ws/src/fr3_generic_drawing/raster_config/images/" + image_file;
double CONVERSION_FACTOR;
double DRAWING_HEIGHT = pen_height;
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

template <typename T>
T clamp(T val, T low, T high)
{
    return std::max(low, std::min(val, high));
}

geometry_msgs::msg::Quaternion orientation()
{
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, 0);
    q.normalize();
    return tf2::toMsg(q);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("test_node", node_options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();

    moveit::planning_interface::MoveGroupInterface mg(node, "fr3_arm");
    mg.startStateMonitor();
    mg.setPlanningTime(10);
    mg.setMaxVelocityScalingFactor(0.2);
    mg.setMaxAccelerationScalingFactor(0.2);
    mg.setPoseReferenceFrame("fr3_link0");
    mg.setEndEffectorLink("fr3_hand_tcp"); // default is 'fr3_link8' pencil_tip
    // Constrain pencil to point down
    moveit_msgs::msg::OrientationConstraint oc;
    oc.link_name = mg.getEndEffectorLink();
    oc.header.frame_id = "fr3_link0";
    oc.orientation = orientation();
    oc.absolute_x_axis_tolerance = 0.1;
    oc.absolute_y_axis_tolerance = 0.1;
    oc.absolute_z_axis_tolerance = 0.1;
    oc.weight = 1.0;

    moveit_msgs::msg::Constraints pc;
    pc.orientation_constraints.push_back(oc);
    mg.setPathConstraints(pc);

    auto stamp_and_execute = [&](moveit_msgs::msg::RobotTrajectory &traj,
                                double v_scale = 0.2,
                                double a_scale = 0.2)
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
    };

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.515;
    pose.position.y = -0.311;
    pose.position.z = 0.215;
    pose.orientation = orientation();

    std::vector<geometry_msgs::msg::Pose> path = {mg.getCurrentPose().pose, pose};
        moveit_msgs::msg::RobotTrajectory traj;
        double start_frac = mg.computeCartesianPath(path, 0.01, 0.0, traj);
        if (start_frac >= 0.95)
        {
            stamp_and_execute(traj);
        }

    rclcpp::shutdown();

    return 0;
}
