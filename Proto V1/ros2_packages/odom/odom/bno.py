import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import math

class BNO055Transformer(Node):

    def __init__(self):
        super().__init__('bno055_transformer')
        self.subscription = self.create_subscription(
            Imu,
            '/bno055/imu',
            self.imu_callback,
            10)
        self.laserSub = self.create_subscription(
            Odometry,
            '/odom_rf2o',
            self.laser_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.laserSub
        
        self.qz = 0.0
        self.qw = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.br = TransformBroadcaster(self)
        self.get_logger().info('Started BNO055 Transformer Node.')
        
        self.timer = self.create_timer(0.1, self.send_tf)
    def send_tf(self):
        # Create a TransformStamped message
        t = TransformStamped()

        # Assign frame ids
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Set the translation to zero or any other known value
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        
        t.transform.rotation.z = self.qz
        t.transform.rotation.w = self.qw
        
        # Broadcast the transform
        self.br.sendTransform(t)
        
    def imu_callback(self, msg):
        # Extract the quaternion from the IMU message
        self.qz = msg.orientation.z
        self.qw = msg.orientation.w

    def laser_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
       
def main(args=None):
    rclpy.init(args=args)

    bno055_transformer = BNO055Transformer()

    rclpy.spin(bno055_transformer)

    # Destroy the node explicitly
    bno055_transformer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
