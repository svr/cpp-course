from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
        ),

        Node(
            package="world_explorer",
            executable="world_explorer_node",
            arguments=["--ros-args", "--log-level", log_level],
        ),

        Node(
            package="world_explorer",
            executable="payload_action_node",
            arguments=["--ros-args", "--log-level", log_level],
        ),
    ])