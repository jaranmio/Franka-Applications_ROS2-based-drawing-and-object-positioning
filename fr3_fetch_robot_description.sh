#!/bin/bash

# Fetch robot_description and robot_description_semantic from move_group
export ROBOT_DESCRIPTION="$(ros2 param get /move_group robot_description | awk 'NR>1')"
export ROBOT_DESCRIPTION_SEMANTIC="$(ros2 param get /move_group robot_description_semantic | awk 'NR>1')"
