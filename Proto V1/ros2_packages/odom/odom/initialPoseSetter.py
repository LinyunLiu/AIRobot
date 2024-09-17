import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped

class TfBroadcaster(Node):

    def __init__(self):
        super().__init__('tf_broadcaster')
        self.br = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            10)
        self.odom_to_base_link = TransformStamped()
        self.initial_pose = None

        # Initialize the odom to base_link transform
        self.odom_to_base_link.header.frame_id = 'odom'
        self.odom_to_base_link.child_frame_id = 'base_link'

        # Create a timer to continuously send the transform
        # self.timer = self.create_timer(0.1, self.broadcast_transform)

    def initialpose_callback(self, msg):
        self.get_logger().info('Received initial pose')
        self.initial_pose = msg.pose.pose
        self.broadcast_transform()
        
    def broadcast_transform(self):
        # If an initial pose has been set, use it to update the transform
        if self.initial_pose:
            self.odom_to_base_link.transform.translation.x = self.initial_pose.position.x
            self.odom_to_base_link.transform.translation.y = self.initial_pose.position.y
            self.odom_to_base_link.transform.translation.z = self.initial_pose.position.z

            self.odom_to_base_link.transform.rotation.x = self.initial_pose.orientation.x
            self.odom_to_base_link.transform.rotation.y = self.initial_pose.orientation.y
            self.odom_to_base_link.transform.rotation.z = self.initial_pose.orientation.z
            self.odom_to_base_link.transform.rotation.w = self.initial_pose.orientation.w

        # Set the timestamp to the current time
        self.odom_to_base_link.header.stamp = self.get_clock().now().to_msg()

        # Broadcast the transform
        self.br.sendTransform(self.odom_to_base_link)

def main(args=None):
    rclpy.init(args=args)
    node = TfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
