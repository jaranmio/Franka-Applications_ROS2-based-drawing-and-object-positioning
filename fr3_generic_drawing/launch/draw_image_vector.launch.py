from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="fr3_generic_drawing",
            executable="draw_image_vector",
            output="screen",
            parameters=[
                # Use same robot description that move_group has
                {"robot_description": os.environ.get("ROBOT_DESCRIPTION", "")},
                {"robot_description_semantic": os.environ.get("ROBOT_DESCRIPTION_SEMANTIC", "")},
                {"use_sim_time": True}, # this is very important for synchronisation during simulation, without which there is a very bad behavior
            ],
        )
    ])

