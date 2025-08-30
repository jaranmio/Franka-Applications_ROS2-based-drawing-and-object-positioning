// pick_place_node.cpp
// Full ROS 2 (Humble) C++ node for pick-and-place using MoveIt2 and Franka FR3 in Isaac Sim

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>

using namespace std::chrono_literals;

class PickPlaceNode : public rclcpp::Node
{
public:
  PickPlaceNode()
  : Node("pick_place_node"),
    move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), "fr3_arm"),
    planning_scene_interface_()
  {
    cube_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/cube_position", 10,
      std::bind(&PickPlaceNode::cubeCallback, this, std::placeholders::_1));

    gripper_client_ = this->create_client<std_srvs::srv::Trigger>("/gripper_command");

    move_group_.setPlanningTime(5.0);
    move_group_.setMaxVelocityScalingFactor(0.2);
    move_group_.setMaxAccelerationScalingFactor(0.2);

    RCLCPP_INFO(this->get_logger(), "PickPlaceNode initialized");
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr cube_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gripper_client_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  void cubeCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received cube at (%.2f, %.2f, %.2f)",
                msg->point.x, msg->point.y, msg->point.z);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "fr3_link0";
    pose.pose.position.x = msg->point.x;
    pose.pose.position.y = msg->point.y;
    pose.pose.position.z = msg->point.z + 0.1;
    pose.pose.orientation.x = 1.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 0.0;

    if (!planToPose(pose, "Pre-grasp")) return;

    pose.pose.position.z = msg->point.z + 0.02;
    if (!planToPose(pose, "Lowering to grasp")) return;

    if (!commandGripper(true)) return;

    pose.pose.position.z = msg->point.z + 0.15;
    if (!planToPose(pose, "Lifting")) return;

    pose.pose.position.x += 0.2;
    if (!planToPose(pose, "Placing")) return;

    if (!commandGripper(false)) return;

    RCLCPP_INFO(this->get_logger(), "Pick-and-place sequence complete");
  }

  bool planToPose(const geometry_msgs::msg::PoseStamped &target, const std::string &desc)
  {
    move_group_.setPoseTarget(target);
    RCLCPP_INFO(this->get_logger(), "Planning: %s", desc.c_str());
    auto success = (move_group_.move() == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) RCLCPP_WARN(this->get_logger(), "Move to %s failed", desc.c_str());
    return success;
  }

  bool commandGripper(bool close)
  {
    if (!gripper_client_->wait_for_service(1s)) {
      RCLCPP_ERROR(this->get_logger(), "Gripper service not available");
      return false;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = gripper_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Gripper command failed");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "%s gripper", close ? "Closed" : "Opened");
    return true;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickPlaceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}