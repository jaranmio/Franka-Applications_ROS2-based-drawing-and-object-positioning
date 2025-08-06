#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <tf2_ros/transform_listener.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_logger");
namespace mtc = moveit::task_constructor;

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <franka_msgs/action/grasp.hpp> 
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/task_constructor/stage.h>

using GraspAction = franka_msgs::action::Grasp;
using GraspGoalHandle = rclcpp_action::ClientGoalHandle<GraspAction>;

const auto& arm_group_name = "fr3_arm";
const auto& hand_group_name = "hand";
const auto& tcp_frame = "fr3_hand_tcp";
const auto& hand_frame = "fr3_hand";
const auto& reference_link = "fr3_link0"; // absolute reference for 'world' coordinate system
const std::string COMPONENTS[] = {
  "object"
};

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  geometry_msgs::msg::PoseStamped getComponentTargetPose(std::string component_name);
  geometry_msgs::msg::PoseStamped getComponentCurrentPose(std::string component_name);
  std::array<float, 3> getBoundingBoxDimens(std::string component_name);
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("pick_place_node", options) }
{
  buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
{
  moveit::planning_interface::PlanningSceneInterface psi;

  auto setComponentStructure = [](std::string component_name, moveit_msgs::msg::CollisionObject& component) {
    if (component_name == "object") {
      component.primitives.resize(1);
      component.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
      component.primitives[0].dimensions = { 0.06, 0.06, 0.1 };       // x-y-z [m]
    } else {
      throw std::invalid_argument("Unknown Object " + component_name + " To Find Current Pose");
    }

  };

  for (std::string component_name : COMPONENTS) {
    geometry_msgs::msg::PoseStamped component_pose = this->getComponentCurrentPose(component_name);
  
    moveit_msgs::msg::CollisionObject component;
    component.id = component_name;
    component.header.frame_id = component_pose.header.frame_id; // should match reference_link

    setComponentStructure(component_name, component);

    component.pose = component_pose.pose;

    psi.applyCollisionObject(component);
  }
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5 /* max_solutions */))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }

  if (task_.solutions().empty()) {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning returned success, but no solutions!");
    task_.introspection().publishTaskDescription();
    return;
  }

  task_.introspection().publishSolution(*task_.solutions().front());

  // Enable real grasp action for execution. Planning state by default
  rclcpp::Parameter planning_param("planning", false);
  node_->set_parameter(planning_param);

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Execution failed");
    RCLCPP_ERROR_STREAM(LOGGER, "MoveIt2 error code for Execution " + std::to_string(result.val));
  } else {
    RCLCPP_INFO_STREAM(LOGGER, "Execution success");
  }

  return;
}

geometry_msgs::msg::Quaternion vertical_orientation() {
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, 0);
    q.normalize();
    return tf2::toMsg(q);
}

geometry_msgs::msg::PoseStamped MTCTaskNode::getComponentTargetPose(std::string component_name) {
  std::string target_frame_name = "target_" + component_name + "_frame";
  geometry_msgs::msg::TransformStamped target_pose_tf = buffer_->lookupTransform(reference_link, target_frame_name, rclcpp::Time(0));

  geometry_msgs::msg::PoseStamped target_pose_msg;
  target_pose_msg.header = target_pose_tf.header; // frame in header should normally correspond to reference_link
  target_pose_msg.pose.orientation = target_pose_tf.transform.rotation;
  target_pose_msg.pose.position.x = target_pose_tf.transform.translation.x;
  target_pose_msg.pose.position.y = target_pose_tf.transform.translation.y;
  target_pose_msg.pose.position.z = target_pose_tf.transform.translation.z;

  return target_pose_msg;
}

geometry_msgs::msg::PoseStamped MTCTaskNode::getComponentCurrentPose(std::string component_name) {
  // should return component pose from computer vision system
  geometry_msgs::msg::PoseStamped current_pose;

  if (component_name == "object") {
    current_pose.header.frame_id = reference_link;
    current_pose.header.stamp = node_->get_clock()->now();

    current_pose.pose.orientation.w = 1.0;
    current_pose.pose.position.x = 0.50;
    current_pose.pose.position.y = -0.25;
    current_pose.pose.position.z = 0.1 / 2.0;
  } else{ 
    throw std::invalid_argument("Unknown Object " + component_name + " To Find Current Pose");
  }

  return current_pose;
}

// returns length along x, y, z respectively
std::array<float, 3> MTCTaskNode::getBoundingBoxDimens(std::string component_name) {
  if (component_name == "object") {
      return { 0.06, 0.06, 0.1 };
  }

  throw std::invalid_argument("Unknown Object " + component_name + " To Find Dimensions");
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("Experimental setup");
  task.loadRobotModel(node_);

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", tcp_frame);

  for (std::string component_name : COMPONENTS) {
    auto pick_place_object = std::make_unique<mtc::SerialContainer>("Pick and Place " + component_name);

    task.properties().exposeTo(pick_place_object->properties(), { "eef", "group", "ik_frame" });
    // clang-format off
    pick_place_object->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });

    mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current " + component_name);
    current_state_ptr = stage_state_current.get();
    pick_place_object->insert(std::move(stage_state_current));

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.0001);

    // clang-format off
    auto stage_open_hand =
        std::make_unique<mtc::stages::MoveTo>("open hand for " + component_name, interpolation_planner);
    // clang-format on
    stage_open_hand->setGroup(hand_group_name);
    stage_open_hand->setGoal("open");
    stage_open_hand->setTimeout(35.0);
    pick_place_object->insert(std::move(stage_open_hand));

    // clang-format off
    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
        "move to pick " + component_name,
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    // clang-format on
    stage_move_to_pick->setTimeout(35.0);
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
    pick_place_object->insert(std::move(stage_move_to_pick));

    // clang-format off
    mtc::Stage* attach_object_stage =
        nullptr;  // Forward attach_object_stage to place pose generator
    // clang-format on

    // This is an example of SerialContainer usage. It's not strictly needed here.
    // In fact, `task` itself is a SerialContainer by default.
    {
      auto grasp = std::make_unique<mtc::SerialContainer>("pick " + component_name);
      task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
      // clang-format off
      grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                            { "eef", "group", "ik_frame" });
      // clang-format on

      {
        // clang-format off
        auto stage =
            std::make_unique<mtc::stages::MoveRelative>("approach " + component_name, cartesian_planner);
        
        // clang-format on
        stage->properties().set("marker_ns", "approach_" + component_name);
        stage->properties().set("link", tcp_frame);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(0.1, 0.15);

        // Set hand downwards direction
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = hand_frame;
        vec.vector.z = 1.0;
        stage->setDirection(vec);
        grasp->insert(std::move(stage));
      }

      /****************************************************
    ---- *               Generate Grasp Pose                *
      ***************************************************/
      {
        // Sample grasp pose
        auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose " + component_name);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "grasp_pose_" + component_name);
        stage->setPreGraspPose("open");
        stage->setObject(component_name);
        stage->setAngleDelta(M_PI / 12);
        stage->setMonitoredStage(current_state_ptr);  // Hook into current state

        // This is the transform from the object frame to the end-effector frame. Pose of object frame relative to end effector frame
        Eigen::Isometry3d grasp_frame_transform;
        Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
                              Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
        grasp_frame_transform.linear() = q.matrix();
        grasp_frame_transform.translation().z() = 0.1 * 0.22;

        // Compute IK
        // clang-format off
        auto wrapper =
            std::make_unique<mtc::stages::ComputeIK>("grasp pose IK " + component_name, std::move(stage));
        // clang-format on
        wrapper->setMaxIKSolutions(8);
        wrapper->setMinSolutionDistance(1.0);
        wrapper->setIKFrame(grasp_frame_transform, tcp_frame);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        grasp->insert(std::move(wrapper));
      }

      {
        // clang-format off
        auto stage =
            std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand," + component_name + ")");
        stage->allowCollisions(component_name,
                              task.getRobotModel()
                                  ->getJointModelGroup(hand_group_name)
                                  ->getLinkModelNamesWithCollisionGeometry(),
                              true);
        // clang-format on
        grasp->insert(std::move(stage));
      }

      {
        auto stage = std::make_unique<mtc::stages::MoveTo>("close hand for " + component_name, interpolation_planner);
        stage->setGroup(hand_group_name);
        float finger_position = (getBoundingBoxDimens(component_name)[0] / 2) - 0.01;
        stage->setGoal(std::map<std::string, double>{{"fr3_finger_joint1", finger_position}, {"fr3_finger_joint2", finger_position}});
        // stage->setGoal("close");
        grasp->insert(std::move(stage));
      }

      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach " + component_name);
        stage->attachObject(component_name, tcp_frame);
        attach_object_stage = stage.get();
        grasp->insert(std::move(stage));
      }

      {
        // clang-format off
        auto stage =
            std::make_unique<mtc::stages::MoveRelative>("lift " + component_name, cartesian_planner);
        // clang-format on
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(0.1, 0.3);
        stage->setIKFrame(tcp_frame);
        stage->properties().set("marker_ns", "lift_" + component_name);

        // Set upward direction
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = reference_link;
        vec.vector.z = 1.0;
        stage->setDirection(vec);
        grasp->insert(std::move(stage));
      }
      pick_place_object->insert(std::move(grasp));
    }

    {
      // clang-format off
      auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
          "move to place " + component_name,
          mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
      // clang-format on
      stage_move_to_place->setTimeout(35.0);
      stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
      pick_place_object->insert(std::move(stage_move_to_place));
    }

    {
      auto place = std::make_unique<mtc::SerialContainer>("place " + component_name);
      task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
      // clang-format off
      place->properties().configureInitFrom(mtc::Stage::PARENT,
                                            { "eef", "group", "ik_frame" });
      // clang-format on

      /****************************************************
    ---- *               Generate Place Pose                *
      ***************************************************/
      {
        // Sample place pose
        auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose " + component_name);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "place_pose_" + component_name);
        stage->setObject(component_name);

        stage->setPose(getComponentTargetPose(component_name));
        stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

        // Compute IK
        // clang-format off
        auto wrapper =
            std::make_unique<mtc::stages::ComputeIK>("place pose IK " + component_name, std::move(stage));
        // clang-format on
        wrapper->setMaxIKSolutions(2);
        wrapper->setMinSolutionDistance(1.0);
        wrapper->setIKFrame(component_name);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        place->insert(std::move(wrapper));
      }

      {
        auto stage = std::make_unique<mtc::stages::MoveTo>("open hand " + component_name, interpolation_planner);
        stage->setGroup(hand_group_name);
        stage->setGoal("open");
        place->insert(std::move(stage));
      }

      {
        // clang-format off
        auto stage =
            std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand," + component_name + ")");
        stage->allowCollisions(component_name,
                              task.getRobotModel()
                                  ->getJointModelGroup(hand_group_name)
                                  ->getLinkModelNamesWithCollisionGeometry(),
                              false);
        // clang-format on
        place->insert(std::move(stage));
      }

      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach " + component_name);
        stage->detachObject(component_name, tcp_frame);
        place->insert(std::move(stage));
      }

      {
        auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat " + component_name, cartesian_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(0.1, 0.3);
        stage->setIKFrame(tcp_frame);
        stage->properties().set("marker_ns", "retreat_" + component_name);

        // Set retreat direction
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = reference_link;
        vec.vector.z = 0.5;
        stage->setDirection(vec);
        place->insert(std::move(stage));
      }
      pick_place_object->insert(std::move(place));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("return home " + component_name, interpolation_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setGoal("ready");
      pick_place_object->insert(std::move(stage));
    }
    
    task.add(std::move(pick_place_object));
  }

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}