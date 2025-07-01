#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iostream>
#include <vector>

const double PEN_HEIGHT = 0.104;
const double X_ORIGIN = 0.44;
const double Y_ORIGIN = 0.0;
const double RAISING_HEIGHT = 0.05;

geometry_msgs::msg::Quaternion vertical_orientation(){
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, 0);
    q.normalize();
    return tf2::toMsg(q);
}

void moveToPose(const geometry_msgs::msg::Pose &pose, moveit::planning_interface::MoveGroupInterface* move_group)
  {
    moveit_msgs::msg::RobotTrajectory traj;
    std::vector<geometry_msgs::msg::Pose> waypoints = {move_group->getCurrentPose().pose, pose};
    double frac = move_group->computeCartesianPath(waypoints, 0.001, 0.0, traj);
    if (frac >= 0.95) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = traj;
        move_group->execute(plan);
    }
  }

void drawDot(double x, double y, moveit::planning_interface::MoveGroupInterface* move_group)
  {
    geometry_msgs::msg::Pose target_pose;

    // Set the target pose based on the current position, adjusting for pen offset
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.orientation = vertical_orientation();

    target_pose.position.z = PEN_HEIGHT + RAISING_HEIGHT; // Lift pen slightly
    moveToPose(target_pose, move_group);

    target_pose.position.z = PEN_HEIGHT; // Pen on Paper
    // Move the robot to the dot position
    moveToPose(target_pose, move_group);

    // Lift pen slightly (to avoid dragging) and move to next dot
    target_pose.position.z = PEN_HEIGHT + RAISING_HEIGHT; // Lift pen slightly
    moveToPose(target_pose, move_group);

  }

void drawGrid(double grid_size_x, double grid_size_y, double spacing, moveit::planning_interface::MoveGroupInterface* move_group)
  {
    // Loop to create the grid of dots
    for (double relative_x = -grid_size_x / 2.0; relative_x <= grid_size_x / 2.0; relative_x += spacing)
    {
      for (double relative_y = -grid_size_y / 2.0; relative_y <= grid_size_y / 2.0; relative_y += spacing)
      {
        // Move the robot to the dot's location, lower the pen, and then raise it slightly between dots
        drawDot(relative_x + X_ORIGIN, relative_y + Y_ORIGIN, move_group);
      }
    }
  }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("fr3_sketch_node");

    moveit::planning_interface::MoveGroupInterface mg(node, "fr3_arm");
    mg.startStateMonitor();
    mg.setPoseReferenceFrame("fr3_link0");
    mg.setEndEffectorLink("fr3_hand_tcp");
    mg.setGoalPositionTolerance(0.0001);
    mg.setGoalOrientationTolerance(0.0001);
    mg.setMaxVelocityScalingFactor(0.025);
    mg.setMaxAccelerationScalingFactor(0.025);
    mg.setPlanningTime(25.0);  

    moveit_msgs::msg::OrientationConstraint oc;
    oc.link_name = mg.getEndEffectorLink();
    oc.header.frame_id = "fr3_link0";
    oc.orientation = vertical_orientation();
    oc.absolute_x_axis_tolerance = 0.015;
    oc.absolute_y_axis_tolerance = 0.015;
    oc.absolute_z_axis_tolerance = 0.015;
    oc.weight = 1.0;
    moveit_msgs::msg::Constraints path_constraints;
    path_constraints.orientation_constraints.push_back(oc);
    mg.setPathConstraints(path_constraints);

    rclcpp::CallbackGroup::SharedPtr timer_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::TimerBase::SharedPtr timer = node->create_wall_timer(std::chrono::milliseconds(0), [node, &mg]() {
        drawGrid(0.24, 0.24, 0.0254, &mg);

        rclcpp::shutdown();
    }, timer_group);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    return 0;
}
