#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <memory>

// Helper function: generate many waypoints forming a square
std::vector<geometry_msgs::msg::Pose> generate_square_waypoints(
    const geometry_msgs::msg::Pose& start_pose,
    double side_length = 0.10,  // 10 cm sides
    double step_size = 0.005    // 5mm per step
)
{
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose pose = start_pose;
  waypoints.push_back(pose);

  int steps_per_side = static_cast<int>(side_length / step_size);

  // Move +X
  for (int i = 0; i < steps_per_side; ++i)
  {
    pose.position.x += step_size;
    waypoints.push_back(pose);
  }

  // Move -Y
  for (int i = 0; i < steps_per_side; ++i)
  {
    pose.position.y -= step_size;
    waypoints.push_back(pose);
  }

  // Move -X
  for (int i = 0; i < steps_per_side; ++i)
  {
    pose.position.x -= step_size;
    waypoints.push_back(pose);
  }

  // Move +Y
  for (int i = 0; i < steps_per_side; ++i)
  {
    pose.position.y += step_size;
    waypoints.push_back(pose);
  }

  return waypoints;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("fr3_draw_square");

  auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");

  if (!planning_scene_monitor->getPlanningScene())
  {
    RCLCPP_ERROR(node->get_logger(), "Planning scene not available");
    rclcpp::shutdown();
    return 1;
  }

  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor();
  planning_scene_monitor->startStateMonitor();

  // Wait for everything (TF, joint states) to be ready
  rclcpp::sleep_for(std::chrono::seconds(3));

  moveit::planning_interface::MoveGroupInterface move_group(node, "fr3_arm");
  move_group.setEndEffectorLink("fr3_hand_tcp");
  move_group.setPoseReferenceFrame("fr3_link0");
  move_group.setMaxVelocityScalingFactor(0.1);
  move_group.setMaxAccelerationScalingFactor(0.1);

  planning_scene_monitor->requestPlanningSceneState();

  // 🛑 NEW: Wait until robot joint states are available
  while (!move_group.startStateMonitor(1.0))
  {
    RCLCPP_WARN(node->get_logger(), "Waiting for current robot state...");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;

  RCLCPP_INFO(node->get_logger(), "Start pose: (%.3f, %.3f, %.3f)",
              start_pose.position.x, start_pose.position.y, start_pose.position.z);

  // Generate square path
  std::vector<geometry_msgs::msg::Pose> waypoints = generate_square_waypoints(start_pose, 0.10, 0.005);

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.005;  // 5mm per interpolation step
  const double jump_threshold = 0.0;

  double fraction = move_group.computeCartesianPath(
      waypoints,
      eef_step,
      jump_threshold,
      trajectory
  );

  if (fraction < 0.99)
    RCLCPP_WARN(node->get_logger(), "Only %.1f%% of Cartesian path was planned", fraction * 100.0);
  else
    RCLCPP_INFO(node->get_logger(), "Cartesian path fully planned!");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;

  move_group.execute(plan);

  rclcpp::shutdown();
  return 0;
}
