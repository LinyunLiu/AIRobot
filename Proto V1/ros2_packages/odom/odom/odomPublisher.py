import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32
import numpy as np

# Publish tf and odometry with subscribed vel_L and vel_R of the wheels

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.get_logger().info('Odometry Publisher Start')
        
        self.left_sub = self.create_subscription(
            Float32,
            'vel_L',
            self.left_callback,
            10
        )
        self.right_sub = self.create_subscription(
            Float32,
            'vel_R',
            self.right_callback,
            10
        )
        
        self.leftVel = 0.0
        self.rightVel = 0.0
        
        self.twistTimer = self.create_timer(0.01, self.setTwist)
        
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.odom_broadcaster = TransformBroadcaster(self)
        
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        self.last_time = self.get_clock().now()
        
        self.timer = self.create_timer(1.0, self.update_odometry)
        self.frameID = 'odom'
        self.childFrameID = 'base_link'
        
    def left_callback(self, msg):
        self.leftVel = msg.data
    
    def right_callback(self, msg):
        self.rightVel = msg.data
    
    def setTwist(self):
        # (linear) m/s
        linear = (self.rightVel +  self.leftVel) / 2
        self.vx = linear * math.cos(self.th)
        self.vy = linear * math.sin(self.th)
        # (angular) rad/s
        self.vth = (self.rightVel -  self.leftVel) / 0.51 # 0.51m (distance between wheels)
        
    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        # delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
        # delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * dt
        delta_x = self.vx * dt
        delta_y = self.vy * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Create quaternion from yaw
        odom_quat = self.create_quaternion_msg_from_yaw(self.th)

        # Publish the transform
        odom_trans = TransformStamped()
        odom_trans.header.stamp = current_time.to_msg()
        odom_trans.header.frame_id = self.frameID
        odom_trans.child_frame_id = self.childFrameID

        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = odom_quat

        self.odom_broadcaster.sendTransform(odom_trans)

        # Publish the odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.frameID 

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_quat

        odom.child_frame_id = self.childFrameID 
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth

        self.odom_pub.publish(odom)

        self.last_time = current_time

    def create_quaternion_msg_from_yaw(self, yaw):
        r = R.from_euler('z', yaw)
        q = r.as_quat()
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

def main(args=None):
    rclpy.init(args=args)

    odometry_publisher = OdometryPublisher()

    rclpy.spin(odometry_publisher)

    odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
