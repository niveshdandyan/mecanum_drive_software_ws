# ~/nd_ws/src/custom_navigation/launch/navigation.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get paths
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    custom_nav_dir = get_package_share_directory('custom_navigation')

    # Declare launch arguments
    map_path = LaunchConfiguration('map', default=os.path.join(custom_nav_dir, 'maps', 'my_map.yaml'))
    params_file = LaunchConfiguration('params_file', default=os.path.join(custom_nav_dir, 'config', 'nav2_params.yaml'))

    # Include the main Nav2 launch file
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'map': map_path,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    # Also include our custom bringup for EKF, LiDAR, etc.
    custom_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('custom_bringup'), 'launch', 'bringup.launch.py')
        )
    )

    return LaunchDescription([
        custom_bringup_launch,
        nav2_bringup_launch
    ])