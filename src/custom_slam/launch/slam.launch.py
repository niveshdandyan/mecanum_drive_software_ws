# ~/nd_ws/src/custom_slam/launch/slam.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the custom SLAM parameters file
    slam_params_file = os.path.join(
        get_package_share_directory('custom_slam'),
        'config', 'mapper_params.yaml')

    # Include your custom bringup launch file
    robot_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('custom_bringup'), 'launch', 'bringup.launch.py')
        )
    )

    # slam_toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': 'false'}],
    )

    return LaunchDescription([
        robot_bringup_launch,
        slam_toolbox_node
    ])