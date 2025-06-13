#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <cmath>

class DrawStarNode : public rclcpp::Node
{
public:
    DrawStarNode()
    : Node("draw_star_node")
    {
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
            std::bind(&DrawStarNode::initialize_move_group, this));
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::TimerBase::SharedPtr timer_;

    void initialize_move_group()
    {
        timer_->cancel();

        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), "fr3_arm");

        move_group_->setMaxVelocityScalingFactor(0.1);
        move_group_->setMaxAccelerationScalingFactor(0.1);

        draw_star();
    }

    void draw_star()
    {
        std::vector<geometry_msgs::msg::Pose> waypoints;

        double base_x = 0.40;
        double base_z = 0.41;
        double base_y = 0.00;

        // ====== Define Star Shape ======

        int num_tips = 5; // 5-point star
        double outer_radius = 0.12;
        double inner_radius = 0.06;

        std::vector<std::pair<double, double>> star_points;

        for (int i = 0; i < 2 * num_tips; ++i)
        {
            double angle = i * M_PI / num_tips;
            double radius = (i % 2 == 0) ? outer_radius : inner_radius;
            double x = radius * sin(angle);
            double y = radius * cos(angle);
            star_points.emplace_back(x, y);
        }

        // Close the star shape
        star_points.push_back(star_points[0]);

        RCLCPP_INFO(this->get_logger(), "Generated %lu star corner points.", star_points.size());

        // Densify: interpolate between points
        const double interpolation_resolution = 0.001;

        for (size_t i = 0; i < star_points.size() - 1; ++i)
        {
            auto [x0, y0] = star_points[i];
            auto [x1, y1] = star_points[i + 1];

            double dx = x1 - x0;
            double dy = y1 - y0;
            double distance = std::sqrt(dx*dx + dy*dy);
            int num_steps = std::max(2, static_cast<int>(distance / interpolation_resolution));

            for (int step = 0; step <= num_steps; ++step)
            {
                double t = static_cast<double>(step) / static_cast<double>(num_steps);
                double x = x0 + t * dx;
                double y = y0 + t * dy;

                geometry_msgs::msg::Pose target_pose;
                target_pose.position.x = base_x + x;
                target_pose.position.y = base_y + y;
                target_pose.position.z = base_z;

                target_pose.orientation.x = 0.707;
                target_pose.orientation.y = 0.0;
                target_pose.orientation.z = 0.0;
                target_pose.orientation.w = 0.707;

                waypoints.push_back(target_pose);
            }
        }

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.001;

        double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if (fraction > 0.95)
        {
            RCLCPP_INFO(this->get_logger(), "Star path computed successfully (%.2f%% achieved). Executing!", fraction * 100.0);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            move_group_->execute(plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute complete star path (%.2f%% achieved).", fraction * 100.0);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DrawStarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
