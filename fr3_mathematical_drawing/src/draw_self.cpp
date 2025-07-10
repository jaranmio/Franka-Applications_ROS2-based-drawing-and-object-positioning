#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <cmath>

class DrawFrankaNode : public rclcpp::Node
{
public:
    DrawFrankaNode()
    : Node("draw_franka_node")
    {
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
            std::bind(&DrawFrankaNode::initialize_move_group, this));
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

        draw_franka();
    }

    void draw_franka()
    {
        std::vector<geometry_msgs::msg::Pose> waypoints;

        double base_x = 0.40; // 40 cm forward
        double base_z = 0.41; // 41 cm height
        double base_y = 0.00; // center Y

        // ====== 100+ outline points: manually defined ======
        std::vector<std::pair<double, double>> outline_points;

        // Upper half curve (0 to +Y side)
        for (int i = 0; i <= 50; ++i) {
            double angle = M_PI * i / 100.0; // 0 to pi/2
            double radius = 0.15; // 15 cm major radius
            double x = radius * sin(angle);
            double y = 0.10 * (1 - cos(angle));
            outline_points.emplace_back(x, y);
        }

        // Lower half curve (+Y to -Y)
        for (int i = 50; i <= 100; ++i) {
            double angle = M_PI * i / 100.0; // pi/2 to pi
            double radius = 0.15;
            double x = radius * sin(angle);
            double y = -0.10 * (1 - cos(angle));
            outline_points.emplace_back(x, y);
        }

        // Small circle at TCP tip
        double tip_radius = 0.02; // 2 cm
        for (int i = 0; i <= 50; ++i) {
            double angle = 2 * M_PI * i / 50.0;
            double x = tip_radius * cos(angle);
            double y = tip_radius * sin(angle) + 0.0;
            outline_points.emplace_back(x, y);
        }

        // ==== End of outline points ====  
        RCLCPP_INFO(this->get_logger(), "Generated %lu primary outline points.", outline_points.size());

        // Densify further (optional)
        const double interpolation_resolution = 0.0015; // 1.5mm step

        for (size_t i = 0; i < outline_points.size() - 1; ++i)
        {
            auto [x0, y0] = outline_points[i];
            auto [x1, y1] = outline_points[i + 1];

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
        const double eef_step = 0.001; // planning resolution

        double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if (fraction > 0.95)
        {
            RCLCPP_INFO(this->get_logger(), "Path computed successfully (%.2f%% achieved). Executing!", fraction * 100.0);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            move_group_->execute(plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute complete Cartesian path (%.2f%%).", fraction * 100.0);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DrawFrankaNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
