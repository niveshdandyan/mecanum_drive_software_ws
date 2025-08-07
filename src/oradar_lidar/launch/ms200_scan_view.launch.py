#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # RViz2 config file
    rviz2_config = os.path.join(
        get_package_share_directory('oradar_lidar'),
        'rviz2',
        'oradar_scan.rviz'
    )

    # LiDAR node launch (reuse your existing ms200_scan.launch.py)
    ordlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('oradar_lidar'),
                'launch',
                'ms200_scan.launch.py'
            )
        )
    )

    # RViz2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_show',
        output='screen',
        arguments=['-d', rviz2_config]
    )

    return LaunchDescription([
        ordlidar_launch,
        rviz2_node,
    ])