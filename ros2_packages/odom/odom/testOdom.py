import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomSubscriber(Node):

    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom_rf2o',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Extract the 2D pose information (x, y, and yaw)
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        orientation_z = msg.pose.pose.orientation.z
        orientation_w = msg.pose.pose.orientation.w

        # Extract the 2D twist information (linear x and angular z)
        linear_x = msg.twist.twist.linear.x
        angular_z = msg.twist.twist.angular.z

        # Print the extracted 2D data
        self.get_logger().info(f'2D Pose - Position: x={position_x}, y={position_y}')
        self.get_logger().info(f'2D Pose - Orientation (yaw, as z and w quaternion): z={orientation_z}, w={orientation_w}')
        self.get_logger().info(f'2D Twist - Linear x: {linear_x}, Angular z: {angular_z}')

def main(args=None):
    rclpy.init(args=args)

    odom_subscriber = OdomSubscriber()

    rclpy.spin(odom_subscriber)

    # Destroy the node explicitly
    odom_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
