import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('next_best_view'),
        'config',
        'octomap_params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='next_best_view',
            executable='octomap_generate',
            name='octomap_generate',
            parameters=[config_file],
            output='screen'
        )
    ])