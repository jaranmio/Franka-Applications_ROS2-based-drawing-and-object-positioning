#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    auto node = rclcpp::Node::make_shared("fr3_rainbow_drawer", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // Create MoveIt interface
    moveit::planning_interface::MoveGroupInterface move_group(node, "fr3_arm");
    move_group.setPlanningTime(10.0);
    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);

    RCLCPP_INFO(node->get_logger(), "Starting to plan rainbow path...");

    // Define the drawing plane
    const double drawing_height = 0.45; // *100cm above ground
    const double center_x = 0.47; // *100cm in front of robot
    const double center_y = 0.27; // centered sideways
    const double inner_radius = 0.07; // *100cm
    const double outer_radius = 0.15; // *100cm
    const double start_angle = -M_PI/4; //  radians
    const double end_angle = -(3*M_PI)/4;    // radians
    const int num_points = 25; // Number of interpolation points

    std::vector<geometry_msgs::msg::Pose> waypoints;

    // Helper function to create poses along an arc
    auto add_arc = [&](double radius, bool reverse) {
        for (int i = 0; i <= num_points; ++i)
        {
            double t = static_cast<double>(i) / num_points;
            double angle = start_angle + t * (end_angle - start_angle);
            if (reverse)
                angle = end_angle - t * (end_angle - start_angle);

            geometry_msgs::msg::Pose pose;
            pose.position.x = center_x + radius * cos(angle);
            pose.position.y = center_y + radius * sin(angle);
            pose.position.z = drawing_height;

            // Set orientation: pencil facing down (aligned with surface)
            tf2::Quaternion orientation;
            orientation.setRPY(M_PI, 0, angle); // Keep tool perpendicular to surface, oriented along arc
            pose.orientation = tf2::toMsg(orientation);

            waypoints.push_back(pose);
        }
    };

    // Helper function to add straight line between two points
    auto add_line = [&](const geometry_msgs::msg::Pose &start, const geometry_msgs::msg::Pose &end) {
        waypoints.push_back(start);
        waypoints.push_back(end);
    };

    // Start from outer arc, moving from left to right
    add_arc(outer_radius, false);

    // Connect outer to inner arc with straight line
    add_line(waypoints.back(), geometry_msgs::msg::Pose());

    geometry_msgs::msg::Pose start_inner_arc;
    start_inner_arc.position.x = center_x + inner_radius * cos(end_angle);
    start_inner_arc.position.y = center_y + inner_radius * sin(end_angle);
    start_inner_arc.position.z = drawing_height;

    tf2::Quaternion orientation_start_inner;
    orientation_start_inner.setRPY(M_PI, 0, end_angle);
    start_inner_arc.orientation = tf2::toMsg(orientation_start_inner);
    waypoints.back() = start_inner_arc; // replace dummy

    // Move along inner arc, moving from right to left (reverse)
    add_arc(inner_radius, true);

    // Connect back to starting point (close the shape)
    geometry_msgs::msg::Pose end_outer_arc;
    end_outer_arc.position.x = center_x + outer_radius * cos(start_angle);
    end_outer_arc.position.y = center_y + outer_radius * sin(start_angle);
    end_outer_arc.position.z = drawing_height;

    tf2::Quaternion orientation_end_outer;
    orientation_end_outer.setRPY(M_PI, 0, start_angle);
    end_outer_arc.orientation = tf2::toMsg(orientation_end_outer);

    add_line(waypoints.back(), end_outer_arc);

    RCLCPP_INFO(node->get_logger(), "Planning Cartesian path with %zu waypoints...", waypoints.size());

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (fraction > 0.9)
    {
        RCLCPP_INFO(node->get_logger(), "Planned %.2f%% of path successfully. Executing...", fraction * 100.0);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        move_group.execute(plan);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to plan Cartesian path. Only %.2f%% achieved.", fraction * 100.0);
    }

    rclcpp::shutdown();
    return 0;
}
