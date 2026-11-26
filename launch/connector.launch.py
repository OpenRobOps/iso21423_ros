"""ros2 launch iso21423_ros connector.launch.py params_file:=/path/to/robot.yaml"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([FindPackageShare("iso21423_ros"), "config", "nav2_default.yaml"]),
            description="Connector parameter YAML (node name iso21423_connector)"),
        Node(package="iso21423_ros", executable="iso21423_connector", name="iso21423_connector",
             output="screen", emulate_tty=True, parameters=[LaunchConfiguration("params_file")]),
    ])
