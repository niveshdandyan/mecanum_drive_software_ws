# ~/nd_ws/src/custom_bringup/launch/bringup.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare and get the use_sim_time launch argument
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
  

    # Paths to packages
    description_pkg_path = get_package_share_directory('custom_robot_description')
    bringup_pkg_path = get_package_share_directory('custom_bringup')
    lidar_pkg_path = get_package_share_directory('oradar_lidar')

    # Path to EKF config file
    ekf_config_path = os.path.join(bringup_pkg_path, 'config', 'ekf.yaml')

    # IMU bias corrector node
    imu_corrector_node = Node(
        package='custom_bringup',
        executable='imu_corrector_node',
        name='imu_corrector_node',
        output='screen',
        parameters=[{
            'calibration_time': 10.0,
            'input_topic': '/imu/data',
            'output_topic': '/imu/data_corrected'
        }]
    )

    # Delayed EKF (robot_localization) node
    delayed_ekf_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
          
                parameters=[ekf_config_path, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('/imu/data', '/imu/data_corrected')
                ]
            )
        ]
    )

    # Robot description launch
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg_path, 'launch', 'display.launch.py')
        ),
       
        launch_arguments={
            'rviz2': 'false', 
            'joint_state_publisher_gui': 'false',
            'use_sim_time': use_sim_time
        }.items()
    )

    # LiDAR launch
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_pkg_path, 'launch', 'ms200_scan.launch.py')
        ),
      
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        declare_use_sim_time_cmd, # <-- ADDED
        imu_corrector_node,
        robot_description_launch,
        lidar_launch,
        delayed_ekf_node
    ])

