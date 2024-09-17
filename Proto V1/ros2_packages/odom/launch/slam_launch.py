from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Launch the RPLIDAR driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('rplidar_ros'),
                    'launch',
                    'rplidar_a1_launch.py'
                )
            ])
        ),
        # Launch the RF2O Laser Odometry node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('rf2o_laser_odometry'),
                    'launch',
                    'rf2o_laser_odometry.launch.py'
                )
            ])
        ),

        # Static transform from map to odom
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '0', '0', '0', '0', '0', '0', '1', 'map', 'odom'
            ],
            output='screen'
        ),

        # Static transform from base_link to laser
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '0', '0', '0', '0', '0', '0', '1', 'base_link', 'laser'
            ],
            output='screen'
        ),
        # Launch the SLAM Toolbox node
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'slam_toolbox', 'async_slam_toolbox_node',
                '--ros-args', '--params-file', 'mapper_params_online_async.yaml'
            ],
            output='screen'
        ),
    ])
