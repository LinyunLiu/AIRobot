import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class StaticTransformFromInitialpose(Node):
    def __init__(self):
        super().__init__('mapToOdomTF')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.handle_initialpose,
            10
        )
        self.static_broadcaster = StaticTransformBroadcaster(self)

    def handle_initialpose(self, msg):
        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"

        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z

        transform.transform.rotation = msg.pose.pose.orientation

        self.static_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = StaticTransformFromInitialpose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
