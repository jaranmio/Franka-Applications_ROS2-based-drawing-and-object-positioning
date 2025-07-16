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
const int SEGMENT_SIZE = 10;
const double SECS_PER_HALF_HOUR = 30 * 60;

template <typename T>
T clamp(T val, T low, T high)
{
    return std::max(low, std::min(val, high));
}

geometry_msgs::msg::Quaternion vertical_orientation()
{
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, 0);
    q.normalize();
    return tf2::toMsg(q);
}

geometry_msgs::msg::Pose image_to_pose(int px, int py, int img_w, int img_h, double z)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = X_ORIGIN + (img_h / 2.0 - py) * CONVERSION_FACTOR;
    pose.position.y = Y_ORIGIN + (img_w / 2.0 - px) * CONVERSION_FACTOR;
    pose.position.z = z;
    pose.orientation = vertical_orientation();
    return pose;
}

// Morphological thinning (Zhang-Suen)
void thinningIteration(cv::Mat &img, int iter)
{
    cv::Mat marker = cv::Mat::zeros(img.size(), CV_8UC1);
    for (int i = 1; i < img.rows - 1; i++)
    {
        for (int j = 1; j < img.cols - 1; j++)
        {
            uchar p2 = img.at<uchar>(i - 1, j);
            uchar p3 = img.at<uchar>(i - 1, j + 1);
            uchar p4 = img.at<uchar>(i, j + 1);
            uchar p5 = img.at<uchar>(i + 1, j + 1);
            uchar p6 = img.at<uchar>(i + 1, j);
            uchar p7 = img.at<uchar>(i + 1, j - 1);
            uchar p8 = img.at<uchar>(i, j - 1);
            uchar p9 = img.at<uchar>(i - 1, j - 1);

            int A = (p2 == 0 && p3 == 255) + (p3 == 0 && p4 == 255) +
                    (p4 == 0 && p5 == 255) + (p5 == 0 && p6 == 255) +
                    (p6 == 0 && p7 == 255) + (p7 == 0 && p8 == 255) +
                    (p8 == 0 && p9 == 255) + (p9 == 0 && p2 == 255);

            int B = (p2 == 255) + (p3 == 255) + (p4 == 255) +
                    (p5 == 255) + (p6 == 255) + (p7 == 255) +
                    (p8 == 255) + (p9 == 255);

            int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
            int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);

            if (img.at<uchar>(i, j) == 255 && A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
                marker.at<uchar>(i, j) = 1;
        }
    }
    img.setTo(0, marker);
}

void thinning(cv::Mat &img)
{
    img /= 255;
    cv::Mat prev = cv::Mat::zeros(img.size(), CV_8UC1);
    cv::Mat diff;
    do
    {
        thinningIteration(img, 0);
        thinningIteration(img, 1);
        cv::absdiff(img, prev, diff);
        img.copyTo(prev);
    } while (cv::countNonZero(diff) > 0);
    img *= 255;
}

// DFS tracing of a single line from a starting point
void dfsTrace(const cv::Mat &binary, cv::Point pt, std::vector<cv::Point> &contour, cv::Mat &visited)
{
    const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};

    std::stack<cv::Point> stack;
    stack.push(pt);
    visited.at<uchar>(pt) = 1;

    while (!stack.empty())
    {
        cv::Point current = stack.top();
        stack.pop();
        contour.push_back(current);

        for (int k = 0; k < 8; ++k)
        {
            cv::Point np(current.x + dx[k], current.y + dy[k]);
            if (np.x < 0 || np.y < 0 || np.x >= binary.cols || np.y >= binary.rows)
                continue;
            if (binary.at<uchar>(np) == 255 && visited.at<uchar>(np) == 0)
            {
                stack.push(np);
                visited.at<uchar>(np) = 1;  // Mark before pushing to avoid re-visits
            }
        }
    }
}

// Extract all centerline paths from a skeletonized image
std::vector<std::vector<cv::Point>> extractPaths(const cv::Mat &binary)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat visited = cv::Mat::zeros(binary.size(), CV_8UC1);

    for (int y = 0; y < binary.rows; ++y)
    {
        for (int x = 0; x < binary.cols; ++x)
        {
            if (binary.at<uchar>(y, x) == 255 && visited.at<uchar>(y, x) == 0)
            {
                std::vector<cv::Point> contour;
                dfsTrace(binary, cv::Point(x, y), contour, visited);
                if (contour.size() > 5)
                {
                    contours.push_back(contour);
                }
            }
        }
    }

    return contours;
}

double euclideanDist(const cv::Point &a, const cv::Point &b)
{
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

// Merge strokes whose endpoints are within a certain pixel threshold
std::vector<std::vector<cv::Point>> mergeCloseContours(std::vector<std::vector<cv::Point>> &contours, double threshold = 5.0)
{
    std::vector<std::vector<cv::Point>> merged;
    std::vector<bool> used(contours.size(), false);

    for (size_t i = 0; i < contours.size(); ++i)
    {
        if (used[i])
            continue;
        std::vector<cv::Point> current = contours[i];
        used[i] = true;

        bool merged_flag;
        do
        {
            merged_flag = false;
            for (size_t j = 0; j < contours.size(); ++j)
            {
                if (used[j])
                    continue;
                const auto &candidate = contours[j];
                double d1 = euclideanDist(current.back(), candidate.front());
                double d2 = euclideanDist(current.back(), candidate.back());
                double d3 = euclideanDist(current.front(), candidate.front());
                double d4 = euclideanDist(current.front(), candidate.back());

                if (d1 < threshold)
                {
                    current.insert(current.end(), candidate.begin(), candidate.end());
                    used[j] = true;
                    merged_flag = true;
                    break;
                }
                else if (d2 < threshold)
                {
                    current.insert(current.end(), candidate.rbegin(), candidate.rend());
                    used[j] = true;
                    merged_flag = true;
                    break;
                }
                else if (d3 < threshold)
                {
                    current.insert(current.begin(), candidate.rbegin(), candidate.rend());
                    used[j] = true;
                    merged_flag = true;
                    break;
                }
                else if (d4 < threshold)
                {
                    current.insert(current.begin(), candidate.begin(), candidate.end());
                    used[j] = true;
                    merged_flag = true;
                    break;
                }
            }
        } while (merged_flag);

        merged.push_back(current);
    }

    return merged;
}

std::vector<std::vector<cv::Point2f>> smoothAndResampleContours(const std::vector<std::vector<cv::Point>> &rawContours, float spacing = 2.0f, int smooth_ksize = 7)
{
    std::vector<std::vector<cv::Point2f>> outputContours;

    for (const auto &contour : rawContours)
    {
        if (contour.size() < 3)
            continue;

        // Interpolate the contour to have roughly equal spacing
        std::vector<cv::Point2f> dense;
        for (size_t i = 1; i < contour.size(); ++i)
        {
            cv::Point2f p1 = contour[i - 1];
            cv::Point2f p2 = contour[i];
            float dist = cv::norm(p2 - p1);
            int steps = std::max(2, static_cast<int>(dist / spacing));
            for (int s = 0; s < steps; ++s)
            {
                float alpha = static_cast<float>(s) / (steps - 1);
                dense.emplace_back((1 - alpha) * p1 + alpha * p2);
            }
        }

        // Smooth with Gaussian kernel
        std::vector<cv::Point2f> smoothed;
        int half_k = smooth_ksize / 2;
        for (size_t i = 0; i < dense.size(); ++i)
        {
            cv::Point2f acc(0.f, 0.f);
            float norm = 0.f;
            for (int j = -half_k; j <= half_k; ++j)
            {
                int idx = clamp<int>(i + j, 0, dense.size() - 1);
                float weight = std::exp(-0.5f * (j * j) / (half_k * half_k));
                acc += dense[idx] * weight;
                norm += weight;
            }
            smoothed.push_back(acc / norm);
        }

        outputContours.push_back(smoothed);
    }

    return outputContours;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("fr3_sketch_node", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();

    moveit::planning_interface::MoveGroupInterface mg(node, "fr3_arm");
    mg.startStateMonitor();
    mg.setPlanningTime(10);
    mg.setMaxVelocityScalingFactor(0.3);
    mg.setPoseReferenceFrame("fr3_link0");
    mg.setEndEffectorLink("fr3_hand_tcp"); // default is 'fr3_link8' pencil_tip
    // Constrain pencil to point down
    moveit_msgs::msg::OrientationConstraint oc;
    oc.link_name = mg.getEndEffectorLink();
    oc.header.frame_id = "fr3_link0";
    oc.orientation = vertical_orientation();
    oc.absolute_x_axis_tolerance = 0.1;
    oc.absolute_y_axis_tolerance = 0.1;
    oc.absolute_z_axis_tolerance = 0.1;
    oc.weight = 1.0;
    moveit_msgs::msg::Constraints pc;
    pc.orientation_constraints.push_back(oc);
    mg.setPathConstraints(pc);

    cv::Mat img = cv::imread(IMAGE_PATH, cv::IMREAD_GRAYSCALE);

    // find conversion factor that maximizes area coverage
    const double image_width_px = img.rows;
    const double image_length_px = img.cols;
    const double image_aspect_ratio = image_length_px / image_width_px;
    
    const double paper_aspect_ratio = paper_length / paper_width;

    double limiting_image_dimension;
    double limiting_paper_dimension;
    
    if ((paper_aspect_ratio > 1 && image_aspect_ratio > 1) || (paper_aspect_ratio < 1 && image_aspect_ratio < 1)) {
        // same aspect ratio
        limiting_paper_dimension = std::min(paper_width, paper_length);
        limiting_image_dimension = std::min(image_width_px, image_length_px);
    } else{
        limiting_paper_dimension = std::min(paper_width, paper_length);
        limiting_image_dimension = std::max(image_width_px, image_length_px);
    }

    CONVERSION_FACTOR = limiting_paper_dimension / limiting_image_dimension;
    CONVERSION_FACTOR *= std::sqrt(percent_coverage / 100);
    RCLCPP_INFO(node->get_logger(), ("CONVERSION FACTOR: " + std::to_string(CONVERSION_FACTOR)).c_str());
    

    cv::Mat binary;
    cv::threshold(img, binary, 128, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
    // only for black/white image. previous: cv::threshold(img, binary, 128, 255, cv::THRESH_BINARY_INV);

    std::vector<std::vector<cv::Point>> contours = extractPaths(binary);
    contours = mergeCloseContours(contours, 5.0);
    std::vector<std::vector<cv::Point2f>> contoursFinal = smoothAndResampleContours(contours);

    // Helper code to see extracted contours
    std::vector<std::vector<cv::Point>> contoursToDraw;
    for (const auto &smoothStroke : contoursFinal)
    {
        if (smoothStroke.empty())
            continue;
        std::vector<cv::Point> converted;
        for (const auto &pt : smoothStroke)
            converted.push_back(cv::Point(cvRound(pt.x), cvRound(pt.y)));
        contoursToDraw.push_back(converted);
    }


    cv::Mat savedImage = cv::Mat(binary.size(), binary.type(), cv::Scalar(255, 255, 255));
    cv::drawContours(savedImage, contoursToDraw, -1, cv::Scalar(0, 255, 0), 1);
    auto saveDir = "/home/qpaig/my_ros2_ws/src/fr3_generic_drawing/src/contours.png";
    cv::imwrite(saveDir, savedImage);

    auto START = std::chrono::high_resolution_clock::now(); // start clock
    double half_hours_tracking = 0; // variable to make sure we reduce exactly once per half hour

    for (const auto &stroke : contoursFinal)
    {

        if (stroke.size() < 2)
            continue;

        // Plan to the starting pose
        geometry_msgs::msg::Pose lift_start = image_to_pose(stroke.front().x, stroke.front().y, img.cols, img.rows, Z_PENCIL_RAISED);
        std::vector<geometry_msgs::msg::Pose> lift_start_path = {mg.getCurrentPose().pose, lift_start};
        moveit_msgs::msg::RobotTrajectory lift_start_traj;
        double start_frac = mg.computeCartesianPath(lift_start_path, 0.01, 0.0, lift_start_traj);
        if (start_frac >= 0.95)
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = lift_start_traj;
            mg.execute(plan);
        }

        for (size_t i = 0; i < stroke.size(); i += SEGMENT_SIZE)
        {
            auto NOW = std::chrono::high_resolution_clock::now();
            const double ELAPSED_TIME_SECS = (NOW - START).count();
            const double HALF_HOURS_ELASPED = std::floor(ELAPSED_TIME_SECS / SECS_PER_HALF_HOUR);
            
            if (HALF_HOURS_ELASPED < half_hours_tracking) { // reduce height only once per half hour
                const double HEIGHT_ADJUSTMENT = HALF_HOURS_ELASPED * height_rate;
                DRAWING_HEIGHT -= HEIGHT_ADJUSTMENT;
                RCLCPP_INFO(node->get_logger(), ("NEW HEIGHT SET: " + std::to_string(DRAWING_HEIGHT) + "m").c_str());
                half_hours_tracking++;
            }

            size_t end = std::min(i + SEGMENT_SIZE, stroke.size());
            std::vector<geometry_msgs::msg::Pose> segment;
            for (size_t j = i; j < end; ++j)
            {
                int x = stroke[j].x; // clamp(stroke[j].x, 0, img.cols - 1);
                int y = stroke[j].y; // clamp(stroke[j].y, 0, img.rows - 1);
                segment.push_back(image_to_pose(x, y, img.cols, img.rows, Z_PENCIL_DOWN));
            }
            if (segment.size() < 2)
                continue;

            moveit_msgs::msg::RobotTrajectory seg_traj;
            double frac = mg.computeCartesianPath(segment, 0.005, 0.0, seg_traj);
            if (frac >= 0.95)
            {
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                plan.trajectory_ = seg_traj;
                mg.execute(plan);
            }
            else
            {
                mg.setPoseTarget(segment.back());
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                if (mg.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
                {
                    mg.execute(plan);
                }
                else
                {
                }
            }
        }

        // Plan to the next start pose
        geometry_msgs::msg::Pose lift_end = image_to_pose(stroke.back().x, stroke.back().y, img.cols, img.rows, Z_PENCIL_RAISED);
        std::vector<geometry_msgs::msg::Pose> lift_end_path = {mg.getCurrentPose().pose, lift_end};
        moveit_msgs::msg::RobotTrajectory lift_end_traj;
        double end_frac = mg.computeCartesianPath(lift_end_path, 0.01, 0.0, lift_end_traj);
        if (end_frac >= 0.95)
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = lift_end_traj;
            mg.execute(plan);
        }
    }

    auto NOW = std::chrono::high_resolution_clock::now();
    const double ELAPSED_TIME_SECS = (NOW - START).count();

    RCLCPP_INFO(node->get_logger(), ("TOTAL DRAWING TIME: " + std::to_string(ELAPSED_TIME_SECS) + "secs").c_str());

    rclcpp::shutdown();

    return 0;
}
