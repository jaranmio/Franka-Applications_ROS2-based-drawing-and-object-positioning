// perception_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CubePerceptionNode : public rclcpp::Node
{
public:
  CubePerceptionNode()
  : Node("cube_perception_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/rgb", 10, std::bind(&CubePerceptionNode::rgbCallback, this, std::placeholders::_1));

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/depth", 10, std::bind(&CubePerceptionNode::depthCallback, this, std::placeholders::_1));

    info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", 10, std::bind(&CubePerceptionNode::infoCallback, this, std::placeholders::_1));

    cube_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/cube_position", 10);
  }

private:
  sensor_msgs::msg::CameraInfo::SharedPtr cam_info_;
  cv::Mat last_depth_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr cube_pub_;

  void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    cam_info_ = msg;
  }

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      last_depth_ = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Depth image conversion failed: %s", e.what());
    }
  }

  void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!cam_info_ || last_depth_.empty()) return;

    cv::Mat rgb;
    try {
      rgb = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "RGB image conversion failed: %s", e.what());
      return;
    }

    cv::Mat hsv;
    cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);

    // Define green color range
    cv::Scalar lower_green(50, 100, 100);
    cv::Scalar upper_green(70, 255, 255);
    cv::Mat mask;
    cv::inRange(hsv, lower_green, upper_green, mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return;

    auto largest = *std::max_element(contours.begin(), contours.end(),
      [](const auto &a, const auto &b) { return cv::contourArea(a) < cv::contourArea(b); });

    cv::Moments M = cv::moments(largest);
    if (M.m00 == 0) return;
    int u = static_cast<int>(M.m10 / M.m00);
    int v = static_cast<int>(M.m01 / M.m00);

    if (v < 0 || v >= last_depth_.rows || u < 0 || u >= last_depth_.cols)
      return;

    float depth = last_depth_.at<float>(v, u);
    if (depth <= 0 || std::isnan(depth)) return;

    double fx = cam_info_->k[0];
    double fy = cam_info_->k[4];
    double cx = cam_info_->k[2];
    double cy = cam_info_->k[5];

    geometry_msgs::msg::PointStamped pt_cam;
    pt_cam.header = msg->header;
    pt_cam.point.x = (u - cx) * depth / fx;
    pt_cam.point.y = (v - cy) * depth / fy;
    pt_cam.point.z = depth;

    try {
      auto pt_base = tf_buffer_.transform(pt_cam, "fr3_link0", tf2::durationFromSec(0.5));
      cube_pub_->publish(pt_base);
      RCLCPP_INFO(this->get_logger(), "Cube at base frame: [%.3f, %.3f, %.3f]",
                  pt_base.point.x, pt_base.point.y, pt_base.point.z);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CubePerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
