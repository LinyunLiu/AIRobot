from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rplidar_launch_file_dir = os.path.join(get_package_share_directory('rplidar_ros'), 'launch')
    rplidar_launch_file = os.path.join(rplidar_launch_file_dir, 'rplidar_a1_launch.py')
    
    ekf_dir  = os.path.join(get_package_share_directory('odom'), 'ekf')
    
    return LaunchDescription([
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_file),
        ),

        #Launch odom_publisher node
        Node(
            package='odom',
            executable='odom_publisher',
            name='odom_publisher_node',
            output='screen',
        ),
        
        # Launch bno055 IMU node
        Node(
            package='ros2_bno055',
            executable='bno055',
            name='bno055_node',
            output='screen',
        ),
        
        #Launch rf2o_laser_odometry node
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry_node',
            output='screen',
            parameters=[{
                    'laser_scan_topic' : '/scan',
                    'odom_topic' : '/odom_rf2o',
                    'publish_tf' : False,
                    'base_frame_id' : 'laser',
                    'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'freq' : 10.0}],
        ),
        
        # Launch EKF localization node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(ekf_dir, 'ekf_config.yaml')], 
            remappings=[
                ('/imu', '/bno055/imu'), 
                ('/odom', '/odom_rf2o'),
                ('/odom1', '/odom') 
            ]
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
