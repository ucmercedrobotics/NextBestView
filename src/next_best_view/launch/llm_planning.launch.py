from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("next_best_view"), "launch", "moveit.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "robot_ip": "192.168.1.10",
            "use_fake_hardware": "false",
            "launch_rviz": "true",
            "vision": "true",
        }.items(),
    )

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

    # Start Kinova Vision with Depth Registration
    kinova_vision = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "kinova_vision",
            "kinova_vision.launch.py",
            "depth_registration:=true",
        ],
        output="screen",
    )

    # Run Point Cloud Saver
    point_cloud_saver = ExecuteProcess(
        cmd=["ros2", "run", "next_best_view", "point_cloud_saver"], output="screen"
    )

    # Run Mission Interface
    mission_interface = ExecuteProcess(
        cmd=["ros2", "run", "mission_interface", "mission_interface"], output="screen"
    )

    # Run Object Detection
    object_detection = ExecuteProcess(
        cmd=["ros2", "run", "next_best_view", "object_detection.py"], output="screen"
    )

    return LaunchDescription(
        [
            moveit_launch,
            moveto,
            nextbestview,
            kinova_vision,
            point_cloud_saver,
            mission_interface,
            object_detection,
        ]
    )
