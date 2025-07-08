#!/bin/bash

# Extract key parameters from /move_group and export them
export ROBOT_DESCRIPTION="$(ros2 param get /move_group robot_description | awk 'NR>1')"
export ROBOT_DESCRIPTION_SEMANTIC="$(ros2 param get /move_group robot_description_semantic | awk 'NR>1')"

# Manually assemble the kinematics map
# Fetch kinematics solver values for 'fr3_arm' group from move_group
SOLVER=$(ros2 param get /move_group fr3_arm.kinematics_solver)
TIMEOUT=$(ros2 param get /move_group fr3_arm.kinematics_solver_timeout)
RESOLUTION=$(ros2 param get /move_group fr3_arm.kinematics_solver_search_resolution)

export ROBOT_DESCRIPTION_KINEMATICS="$(echo "{'fr3_arm': {'kinematics_solver': '$SOLVER', 'kinematics_solver_timeout': $TIMEOUT, 'kinematics_solver_search_resolution': $RESOLUTION}}")"

