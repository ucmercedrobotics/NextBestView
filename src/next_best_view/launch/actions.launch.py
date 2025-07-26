import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    print(use_sim_time)

    moveit_config = MoveItConfigsBuilder(
        "name", package_name="kinova_gen3_6dof_robotiq_2f_85_moveit_config"
    ).to_moveit_configs()

    # Get the package share directory for next_best_view
    package_share_directory = get_package_share_directory('next_best_view')
    # Path to the YAML parameter file
    params_file = os.path.join(package_share_directory, 'config', 'nextbestview_params.yaml')

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
            {"use_sim_time": use_sim_time},  # Enable simulation time for Gazebo
        ],
    )
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
            params_file,  # Load the YAML parameters file
            {"use_sim_time": use_sim_time},  # Enable simulation time for Gazebo
        ],
    )

    return LaunchDescription([moveto, nextbestview])
