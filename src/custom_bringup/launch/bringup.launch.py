# ~/nd_ws/src/custom_bringup/launch/bringup.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Paths to packages
    description_pkg_path = get_package_share_directory('custom_robot_description')
    bringup_pkg_path = get_package_share_directory('custom_bringup')
    lidar_pkg_path = get_package_share_directory('oradar_lidar')

    # EKF (robot_localization) node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(bringup_pkg_path, 'config', 'ekf.yaml')]
    )

    # Include the robot description launch file (from custom_robot_description)
    # This starts robot_state_publisher
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg_path, 'launch', 'display.launch.py')
        ),
        # Disable the GUI components for this context
        launch_arguments={'rviz2': 'false', 'joint_state_publisher_gui': 'false'}.items()
    )

    # Include the LiDAR launch file
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_pkg_path, 'launch', 'ms200_scan.launch.py')
        )
    )

    return LaunchDescription([
        ekf_node,
        robot_description_launch,
        lidar_launch
    ])