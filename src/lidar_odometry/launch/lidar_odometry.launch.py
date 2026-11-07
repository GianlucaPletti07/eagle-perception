from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('lidar_odometry')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([package_share, 'config', 'lidar_odometry.yaml']),
            description='YAML file with the node parameters'),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Open RViz with the provided configuration'),
        Node(
            package='lidar_odometry',
            executable='lidar_odometry_node',
            name='lidar_odometry',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([package_share, 'rviz', 'lidar_odometry.rviz'])],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
