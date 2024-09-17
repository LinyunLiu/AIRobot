import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import open3d as o3d

class LidarTransformNode(Node):
    def __init__(self):
        super().__init__('lidar_transform_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        # self.subscription  # prevent unused variable warning

        self.br = TransformBroadcaster(self)
        self.previous_cloud = None

    def scan_callback(self, msg):
        # Convert LaserScan to Open3D PointCloud
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        points = np.column_stack((x, y, np.zeros_like(x)))  # Z is zero for 2D

        # Create Open3D PointCloud
        current_cloud = o3d.geometry.PointCloud()
        current_cloud.points = o3d.utility.Vector3dVector(points)

        if self.previous_cloud is None:
            self.previous_cloud = current_cloud
            return

        # Apply G-ICP using Open3D
        threshold = 1.0
        trans_init = np.eye(4)
        reg_p2p = o3d.pipelines.registration.registration_generalized_icp(
            current_cloud, self.previous_cloud, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationForGeneralizedICP())

        transformation = reg_p2p.transformation

        # Update previous cloud
        self.previous_cloud = current_cloud

        # Extract 2D translation and rotation (yaw)
        translation = transformation[:2, 3]
        yaw = np.arctan2(transformation[1, 0], transformation[0, 0])

        # Convert yaw angle to quaternion for 2D rotation
        quat = self.yaw_to_quaternion(yaw)

        # Create TransformStamped message
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'map'
        transform_stamped.child_frame_id = 'base_link'
        transform_stamped.transform.translation.x = translation[0]
        transform_stamped.transform.translation.y = translation[1]
        transform_stamped.transform.translation.z = 0.0  # No Z translation in 2D
        transform_stamped.transform.rotation.x = quat[0]
        transform_stamped.transform.rotation.y = quat[1]
        transform_stamped.transform.rotation.z = quat[2]
        transform_stamped.transform.rotation.w = quat[3]

        # Publish the transform
        self.br.sendTransform(transform_stamped)

    def yaw_to_quaternion(self, yaw):
        """
        Convert a yaw angle (rotation around the Z-axis) to a quaternion.
        """
        return [0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)]

def main(args=None):
    rclpy.init(args=args)
    node = LidarTransformNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
