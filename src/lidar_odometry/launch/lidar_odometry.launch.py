from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Follow the /clock published by "ros2 bag play --clock"'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('lidar_odometry'), 'config', 'lidar_odometry.yaml']),
            description='YAML file with the node parameters'),
        Node(
            package='lidar_odometry',
            executable='lidar_odometry_node',
            name='lidar_odometry',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
    ])