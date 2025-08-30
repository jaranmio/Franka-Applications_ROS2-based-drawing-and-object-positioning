cmake_minimum_required(VERSION 3.5)
project(fr3_mathematical_drawing)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(draw_self src/draw_self.cpp)
add_executable(draw_square src/draw_square.cpp)
add_executable(draw_curve src/draw_curve.cpp)
add_executable(draw_star src/draw_star.cpp)

ament_target_dependencies(draw_self
  rclcpp
  moveit_ros_planning_interface
  tf2_geometry_msgs
  OpenCV
)

ament_target_dependencies(draw_square
  rclcpp
  moveit_ros_planning_interface
  tf2_geometry_msgs
  OpenCV
)

ament_target_dependencies(draw_curve
  rclcpp
  moveit_ros_planning_interface
  tf2_geometry_msgs
  OpenCV
)

ament_target_dependencies(draw_star
  rclcpp
  moveit_ros_planning_interface
  tf2_geometry_msgs
  OpenCV
)

target_link_libraries(draw_self
  tinyxml2
)

install(TARGETS draw_self
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS draw_square
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS draw_curve
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS draw_star
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
