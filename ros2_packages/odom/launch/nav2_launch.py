from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

# ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=False map:=hallway.yaml

def generate_launch_description():
    return LaunchDescription([
        
        # RPLIDAR A1 launch
        ExecuteProcess(
            cmd=['ros2', 'launch', 'rplidar_ros', 'rplidar_a1_launch.py'],
            name='rplidar_a1_launch'
        ),

        # Run the static transform publisher for base_link to laser
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '0', '0', '0', '0', '0', '0', '1', 'base_link', 'laser'],
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['ros2', 'run', 'odom', 'mapToOdomTF'],
            output='screen'
        ),
        
        Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[{
                    'laser_scan_topic' : '/scan',
                    'odom_topic' : '/odom_rf2o',
                    'publish_tf' : True,
                    'base_frame_id' : 'base_link',
                    'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'freq' : 10.0}],
            ),
        
    ])
