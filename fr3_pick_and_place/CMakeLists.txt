cmake_minimum_required(VERSION 3.8)
project(fr3_pick_and_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(perception_node src/perception_node.cpp)
ament_target_dependencies(perception_node rclcpp std_msgs sensor_msgs cv_bridge image_transport tf2_ros tf2_geometry_msgs geometry_msgs moveit_ros_planning_interface)

add_executable(pick_place_node src/pick_place_node.cpp)
ament_target_dependencies(pick_place_node rclcpp std_msgs sensor_msgs cv_bridge image_transport tf2_ros tf2_geometry_msgs geometry_msgs moveit_ros_planning_interface)

install(TARGETS
  perception_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  pick_place_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
