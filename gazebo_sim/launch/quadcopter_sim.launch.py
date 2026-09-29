#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


PACKAGE_NAME = 'stm32_h750_gazebo_sim'


def generate_launch_description():
    world = LaunchConfiguration('world')
    package_share = FindPackageShare(PACKAGE_NAME)
    gazebo_share = FindPackageShare('ros_gz_sim')
    model_path = PathJoinSubstitution([package_share, 'models'])
    bridge_config = PathJoinSubstitution([package_share, 'config', 'ros_gz_bridge.yaml'])
    gazebo_launch = PathJoinSubstitution([gazebo_share, 'launch', 'gz_sim.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([package_share, 'worlds', 'test.world.sdf']),
            description='Gazebo world file',
        ),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'gz_args': [world, ' -r'],
                'on_exit_shutdown': 'true',
            }.items(),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            parameters=[{'config_file': bridge_config}],
            output='screen',
        ),
        Node(
            package=PACKAGE_NAME,
            executable='motor_adapter.py',
            name='motor_adapter',
            output='screen',
        ),
    ])
