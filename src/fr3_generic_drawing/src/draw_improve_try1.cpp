// fr3_sketch_drawer.cpp — Segment-fragmented drawing for robustness
// Breaks strokes into small fragments and skips segments with jumps
// Now batches all stroke segments into a single trajectory per contour for speed

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <moveit/robot_state/robot_state.h>
#include <cmath>

const std::string IMAGE_PATH = "/home/qpaig/my_ros2_ws/src/fr3_generic_drawing/src/flower_outline.jpg";
const double CONVERSION_FACTOR = 0.0003;
const double DRAWING_HEIGHT = 0.405;
const double TIP_OFFSET = 0.015;
const double Z_PENCIL_DOWN = DRAWING_HEIGHT + TIP_OFFSET;
const double Z_PENCIL_RAISED = Z_PENCIL_DOWN + 0.02;
const double X_ORIGIN = 0.4;
const double Y_ORIGIN = 0.0;
const int SEGMENT_SIZE = 5;
const double EEF_STEP = 0.01;  // Use larger end-effector step for faster Cartesian planning

template <typename T>
T clamp(T val, T low, T high) {
    return std::max(low, std::min(val, high));
}

geometry_msgs::msg::Quaternion orientation() {
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, 0);
    q.normalize();
    return tf2::toMsg(q);
}

geometry_msgs::msg::Pose image_to_pose(int px, int py, int img_w, int img_h, double z) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = X_ORIGIN + (img_h / 2.0 - py) * CONVERSION_FACTOR;
    pose.position.y = Y_ORIGIN + (img_w / 2.0 - px) * CONVERSION_FACTOR;
    pose.position.z = z;
    pose.orientation = orientation();
    return pose;
}

// Append trajectory 'traj' to 'combined_traj', adjusting time_from_start for continuity
void appendTrajectory(moveit_msgs::msg::RobotTrajectory &combined_traj,
                      const moveit_msgs::msg::RobotTrajectory &traj,
                      uint64_t &offset_ns) {
    // Copy joint names if combined trajectory is empty
    if (combined_traj.joint_trajectory.joint_names.empty()) {
        combined_traj.joint_trajectory.joint_names = traj.joint_trajectory.joint_names;
    }
    // Determine if we should skip the first point of traj to avoid duplicate pose 
    // (if it matches the end of the previous trajectory)
    bool skip_first = false;
    if (!combined_traj.joint_trajectory.points.empty() && !traj.joint_trajectory.points.empty()) {
        skip_first = true;
    }
    // Append each point from traj to combined_traj with time offset
    for (size_t idx = (skip_first ? 1 : 0); idx < traj.joint_trajectory.points.size(); ++idx) {
        trajectory_msgs::msg::JointTrajectoryPoint point = traj.joint_trajectory.points[idx];
        // Calculate time from start in nanoseconds for this point plus offset
        uint64_t point_ns = (uint64_t)point.time_from_start.sec * 1000000000ULL + point.time_from_start.nanosec;
        uint64_t new_time_ns = offset_ns + point_ns;
        // Fill new time into point.time_from_start
        point.time_from_start.sec = (int32_t)(new_time_ns / 1000000000ULL);
        point.time_from_start.nanosec = (new_time_ns % 1000000000ULL);
        combined_traj.joint_trajectory.points.push_back(point);
    }
    // Update offset to last time of combined_traj
    if (!combined_traj.joint_trajectory.points.empty()) {
        const auto &last_point = combined_traj.joint_trajectory.points.back();
        offset_ns = (uint64_t)last_point.time_from_start.sec * 1000000000ULL + last_point.time_from_start.nanosec;
    }
}

// Set the MoveGroup start state to the end of the given trajectory 
void setStartStateToTrajectoryEnd(const moveit_msgs::msg::RobotTrajectory &traj,
                                  moveit::planning_interface::MoveGroupInterface &mg) {
    if (traj.joint_trajectory.points.empty()) return;
    // Copy current robot state and update joint values to trajectory end
    moveit::core::RobotState state(*mg.getCurrentState());
    const auto &joint_names = traj.joint_trajectory.joint_names;
    const auto &last_point = traj.joint_trajectory.points.back();
    for (size_t idx = 0; idx < joint_names.size(); ++idx) {
        state.setJointPositions(joint_names[idx], &last_point.positions[idx]);
    }
    mg.setStartState(state);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("fr3_sketch_node");
    moveit::planning_interface::MoveGroupInterface mg(node, "fr3_arm");
    mg.setPlanningTime(10);
    mg.setMaxVelocityScalingFactor(0.3);
    mg.setPoseReferenceFrame("base");
    mg.setEndEffectorLink("fr3_hand_tcp");

    // Constrain pencil to point down
    moveit_msgs::msg::OrientationConstraint oc;
    oc.link_name = mg.getEndEffectorLink();
    oc.header.frame_id = "base";
    oc.orientation = orientation();
    oc.absolute_x_axis_tolerance = 0.1;
    oc.absolute_y_axis_tolerance = 0.1;
    oc.absolute_z_axis_tolerance = 0.1;
    oc.weight = 1.0;
    moveit_msgs::msg::Constraints pc;
    pc.orientation_constraints.push_back(oc);
    mg.setPathConstraints(pc);

    cv::Mat img = cv::imread(IMAGE_PATH, cv::IMREAD_GRAYSCALE);
    cv::Mat binary;
    cv::threshold(img, binary, 128, 255, cv::THRESH_BINARY);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    for (const auto& stroke : contours) {
        if (stroke.size() < 2) continue;
        mg.setStartStateToCurrentState();

        // Plan approach (lift) to the start of the stroke
        geometry_msgs::msg::Pose lift_start_pose = image_to_pose(stroke.front().x, stroke.front().y,
                                                                 img.cols, img.rows, Z_PENCIL_RAISED);
        std::vector<geometry_msgs::msg::Pose> approach_waypoints;
        approach_waypoints.push_back(mg.getCurrentPose().pose);
        approach_waypoints.push_back(lift_start_pose);
        moveit_msgs::msg::RobotTrajectory approach_traj;
        double approach_fraction = mg.computeCartesianPath(approach_waypoints, EEF_STEP, 0.0, approach_traj);
        if (approach_fraction < 0.95) {
            // If Cartesian path to approach fails, use standard planning as fallback
            mg.setPoseTarget(lift_start_pose);
            moveit::planning_interface::MoveGroupInterface::Plan alt_plan;
            if (mg.plan(alt_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_WARN(node->get_logger(), "Could not plan approach to stroke start, skipping stroke");
                continue;  // skip this stroke if we cannot reach the start position
            }
            approach_traj = alt_plan.trajectory_;
        }

        // Initialize combined trajectory for this stroke
        moveit_msgs::msg::RobotTrajectory combined_traj;
        uint64_t offset_ns = 0ULL;
        appendTrajectory(combined_traj, approach_traj, offset_ns);
        // Set the virtual start state to the end of the approach (above start of stroke)
        setStartStateToTrajectoryEnd(approach_traj, mg);

        bool drawn_any_segment = false;
        bool stroke_failed = false;
        geometry_msgs::msg::Pose last_draw_pose;  // track last drawn pose (with pencil down)

        // Plan and append each small segment of the stroke (SEGMENT_SIZE points per segment)
        for (size_t i = 0; i < stroke.size(); i += SEGMENT_SIZE) {
            size_t end_idx = std::min(i + SEGMENT_SIZE, stroke.size());
            std::vector<geometry_msgs::msg::Pose> segment_waypoints;
            for (size_t j = i; j < end_idx; ++j) {
                int x = stroke[j].x; // clamp(stroke[j].x, 0, img.cols - 1);
                int y = stroke[j].y; // clamp(stroke[j].y, 0, img.rows - 1);
                segment_waypoints.push_back(image_to_pose(x, y, img.cols, img.rows, Z_PENCIL_DOWN));
            }
            if (segment_waypoints.size() < 2) continue;

            moveit_msgs::msg::RobotTrajectory seg_traj;
            double frac = mg.computeCartesianPath(segment_waypoints, EEF_STEP, 0.0, seg_traj);
            if (frac < 0.95) {
                // If Cartesian path fails for this segment, attempt normal planning to last pose
                mg.setPoseTarget(segment_waypoints.back());
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                if (mg.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_WARN(node->get_logger(), "Segment starting at index %zu is unreachable, aborting stroke early", i);
                    stroke_failed = true;
                    break;
                }
                seg_traj = plan.trajectory_;
            }

            // Append this segment trajectory to the combined trajectory
            appendTrajectory(combined_traj, seg_traj, offset_ns);
            // Update planning start state to end of this segment
            setStartStateToTrajectoryEnd(seg_traj, mg);
            drawn_any_segment = true;
            last_draw_pose = segment_waypoints.back();
        }

        if (stroke_failed) {
            if (!drawn_any_segment) {
                // No segment drawn at all, skip executing this stroke entirely
                continue;
            }
            // Some segments were drawn before failure; raise pen from last drawn point and execute partial trajectory
            geometry_msgs::msg::Pose raise_pose = last_draw_pose;
            raise_pose.position.z = Z_PENCIL_RAISED;
            std::vector<geometry_msgs::msg::Pose> raise_waypoints;
            raise_waypoints.push_back(raise_pose);
            moveit_msgs::msg::RobotTrajectory retract_traj;
            double lift_frac = mg.computeCartesianPath(raise_waypoints, EEF_STEP, 0.0, retract_traj);
            if (lift_frac < 0.95) {
                mg.setPoseTarget(raise_pose);
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                if (mg.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                    retract_traj = plan.trajectory_;
                } else {
                    RCLCPP_WARN(node->get_logger(), "Failed to plan retract move, ending stroke without retract");
                    // If we cannot retract, end stroke without raising (will attempt move to next stroke from current position)
                }
            }
            if (!retract_traj.joint_trajectory.points.empty()) {
                appendTrajectory(combined_traj, retract_traj, offset_ns);
            }
            // Execute the combined trajectory (partial stroke) with pen raised at end
            moveit::planning_interface::MoveGroupInterface::Plan combined_plan;
            combined_plan.trajectory_ = combined_traj;
            mg.execute(combined_plan);
            // Continue to next stroke
            continue;
        }

        // If stroke completed normally, plan lift (retraction) from end of stroke
        geometry_msgs::msg::Pose lift_end_pose = image_to_pose(stroke.back().x, stroke.back().y,
                                                               img.cols, img.rows, Z_PENCIL_RAISED);
        std::vector<geometry_msgs::msg::Pose> lift_waypoints;
        lift_waypoints.push_back(lift_end_pose);
        moveit_msgs::msg::RobotTrajectory lift_traj;
        double lift_fraction = mg.computeCartesianPath(lift_waypoints, EEF_STEP, 0.0, lift_traj);
        if (lift_fraction < 0.95) {
            mg.setPoseTarget(lift_end_pose);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (mg.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                lift_traj = plan.trajectory_;
            } else {
                RCLCPP_WARN(node->get_logger(), "Failed to plan final lift for stroke");
                // If final lift fails, proceed without raising (not ideal, but continue)
            }
        }
        if (!lift_traj.joint_trajectory.points.empty()) {
            appendTrajectory(combined_traj, lift_traj, offset_ns);
        }

        // Execute the combined trajectory for this stroke
        moveit::planning_interface::MoveGroupInterface::Plan combined_plan;
        combined_plan.trajectory_ = combined_traj;
        mg.execute(combined_plan);
    }

    rclcpp::shutdown();
    return 0;
}


