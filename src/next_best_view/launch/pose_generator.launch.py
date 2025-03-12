import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Get the package share directory for next_best_view
    package_share_directory = get_package_share_directory('next_best_view')
    # Path to the YAML parameter file
    params_file = os.path.join(package_share_directory, 'config', 'pose_generator_params.yaml')


    # pose_generator Service
    pose_generator = Node(
        name="voxel_map",
        package="next_best_view",
        executable="pose_generator",
        output="screen",
        parameters=[
            params_file  # Load the YAML parameters file
        ],
    )

    return LaunchDescription([pose_generator])