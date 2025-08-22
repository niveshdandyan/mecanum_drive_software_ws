import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


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
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file ],
    )
    
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{'autostart': True},
                    {‘node_names': ['slam_toolbox']}],
    )


    return LaunchDescription([
        robot_bringup_launch,
        slam_toolbox_node,
        lifecycle_manager
    ])

