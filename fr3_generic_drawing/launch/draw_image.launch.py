from launch import LaunchDescription
from launch_ros.actions import Node
import os
import json
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import ast

def cast_value(val):
    if isinstance(val, dict):
        return {
            k: cast_value(v)
            for k, v in val.items()
            if cast_value(v) is not None
        }
    elif isinstance(val, list):
        return [cast_value(v) for v in val if cast_value(v) is not None]
    elif isinstance(val, str):
        val_lower = val.lower().strip()
        if val_lower == "null":
            return None
        if val_lower == "true":
            return True
        if val_lower == "false":
            return False
        try:
            if '.' in val:
                return float(val)
            else:
                return int(val)
        except ValueError:
            pass
        try:
            return ast.literal_eval(val)
        except (ValueError, SyntaxError):
            return val
    else:
        return val

def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("fr3", package_name="fr3_moveit_config")
        .to_moveit_configs()
    )

    moveit_config.planning_pipelines["ompl"]["fr3_arm"]["enforce_constrained_state_space"] = True
    moveit_config.planning_pipelines["ompl"]["fr3_arm"]["projection_evaluator"] = "joints(fr3_joint1,fr3_joint2)"
    moveit_config.planning_pipelines["ompl"]["fr3_arm_hand"]["enforce_constrained_state_space"] = True
    moveit_config.planning_pipelines["ompl"]["fr3_arm_hand"]["projection_evaluator"] = "joints(fr3_joint1,fr3_joint2)"

    return LaunchDescription([
        Node(
            package="fr3_generic_drawing",
            executable="draw_image",
            output="screen",
            parameters=[
                # Use same robot description that move_group has
                # {"robot_description": os.environ.get("ROBOT_DESCRIPTION", "")},
                # {"robot_description_semantic": os.environ.get("ROBOT_DESCRIPTION_SEMANTIC", "")},
                moveit_config.to_dict(),
                {"use_sim_time": True}, # this is very important for synchronisation, without which there is a very bad behavior
            ],
        )
    ])

