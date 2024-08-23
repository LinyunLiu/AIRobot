import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch motor_subscriber2 node from airobot_controller package
        # Node(
        #     package='airobot_controller',
        #     executable='motor_subscriber2',
        #     name='motor_subscriber2',
        #     output='screen'
        # ),
        
        # Launch left_wheel node from odom package
        Node(
            package='odom',
            executable='left_wheel',
            name='left_wheel',
            output='screen'
        ),

        # Launch right_wheel node from odom package
        Node(
            package='odom',
            executable='right_wheel',
            name='right_wheel',
            output='screen'
        ),

        # Launch odom_publisher node from odom package
        Node(
            package='odom',
            executable='odom_publisher',
            name='odom_publisher',
            output='screen'
        ),
    ])
