"""Launch typed shot calculation; no direct belt or initialization commands."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Require an approved calibration file for valid static solutions."""
    default_file = os.path.join(get_package_share_directory('shooter_calculator'),
                                'config', 'calibration.json')
    return LaunchDescription([
        DeclareLaunchArgument('calibration_file', default_value=default_file),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='shooter_calculator', executable='shooter_calculator_node',
             name='shooter_calculator', output='screen', parameters=[{
                 'calibration_file': LaunchConfiguration('calibration_file'),
                 'use_sim_time': LaunchConfiguration('use_sim_time'),
             }]),
    ])
