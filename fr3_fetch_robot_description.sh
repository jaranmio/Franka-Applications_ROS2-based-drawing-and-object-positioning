#!/bin/bash

# Extract key parameters from /move_group and export them
export ROBOT_DESCRIPTION="$(ros2 param get /move_group robot_description | awk 'NR>1')"
export ROBOT_DESCRIPTION_SEMANTIC="$(ros2 param get /move_group robot_description_semantic | awk 'NR>1')"

ros2 param dump /move_group > /home/qpaig/my_ros2_ws/src/move_group_raw.yaml
yq e '.["/move_group"].ros__parameters' /home/qpaig/my_ros2_ws/src/move_group_raw.yaml > /home/qpaig/my_ros2_ws/src/my_params.yaml
