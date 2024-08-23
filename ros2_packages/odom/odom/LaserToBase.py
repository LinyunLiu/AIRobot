import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class TF2Broadcaster(Node):

    def __init__(self):
        super().__init__('LaserToBase')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_tf)

    def broadcast_tf(self):
        t = TransformStamped()

        # Set up header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "laser"
        t.child_frame_id = "base_link"

        # Set translation (assuming no translation here)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Quaternion for no rotation (roll, pitch, yaw = 0, 0, 0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = TF2Broadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

