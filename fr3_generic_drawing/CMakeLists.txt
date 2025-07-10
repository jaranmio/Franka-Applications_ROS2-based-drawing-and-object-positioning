cmake_minimum_required(VERSION 3.8)
project(fr3_generic_drawing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(draw_image src/draw_image.cpp)
ament_target_dependencies(draw_image rclcpp OpenCV moveit_ros_planning_interface geometry_msgs visualization_msgs tf2_ros)

add_executable(draw_image_vector src/draw_image_vector.cpp)
ament_target_dependencies(draw_image_vector rclcpp OpenCV moveit_ros_planning_interface geometry_msgs visualization_msgs tf2_ros)

install(TARGETS
  draw_image
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  draw_image_vector
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
