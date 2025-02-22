import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "name", package_name="kinova_gen3_6dof_robotiq_2f_85_moveit_config"
    ).to_moveit_configs()

    # Moveto Action
    moveto = Node(
        name="moveto",
        package="next_best_view",
        executable="moveto",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )
    # TODO: Check if nextbestview action needed these config files
    # because it is already used wıth moveto action
    # Nextbestview Action
    nextbestview = Node(
        name="nextbestview",
        package="next_best_view",
        executable="nextbestview",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([moveto, nextbestview])
