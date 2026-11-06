from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    rviz = LaunchConfiguration('rviz')

    package_share = FindPackageShare('lidar_odometry')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Follow the /clock published by "ros2 bag play --clock"'),
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
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([package_share, 'rviz', 'lidar_odometry.rviz'])],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz),
        ),
    ])