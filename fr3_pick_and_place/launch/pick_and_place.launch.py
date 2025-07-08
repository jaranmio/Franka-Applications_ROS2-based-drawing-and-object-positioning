import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from os import path
import yaml
import ast

def generate_launch_description():
    # MTC Demo node
    pick_place = Node(
        package="fr3_pick_and_place", 
        executable="pick_place_node",
        output="screen",
        parameters=[{
            "robot_description": os.environ.get("ROBOT_DESCRIPTION", ""),
            "robot_description_semantic": os.environ.get("ROBOT_DESCRIPTION_SEMANTIC", ""),
            "robot_description_kinematics": os.environ.get("ROBOT_DESCRIPTION_KINEMATICS", ""),
            "use_sim_time": True,
        }],
    )

    return LaunchDescription([pick_place])

