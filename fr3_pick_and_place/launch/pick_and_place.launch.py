import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import yaml
import ast
from pathlib import Path

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
    # with open("/home/qpaig/my_ros2_ws/src/my_params.yaml", "r") as f:
    #     raw_params = yaml.safe_load(f)

    # all_params = cast_value(raw_params)

    moveit_config = (
        MoveItConfigsBuilder("fr3", package_name="fr3_moveit_config")
        .to_moveit_configs()
    )

    # MTC Demo node
    pick_place = Node(
        package="fr3_pick_and_place", 
        executable="pick_place_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
            # "robot_description": os.environ.get("ROBOT_DESCRIPTION", ""),
            # "robot_description_semantic": os.environ.get("ROBOT_DESCRIPTION_SEMANTIC", ""),
            "use_sim_time": True,
            'simulate': True
        }],
    )

    return LaunchDescription([pick_place])

