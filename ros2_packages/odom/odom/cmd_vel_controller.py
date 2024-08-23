import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from .motor_driver import *

class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')
        # Subscribe to the /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        # Prevent unused variable warning
        self.subscription  

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities from the Twist message
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Command the motors based on the velocities
        self.command_motors(linear_x, angular_z)

    def command_motors(self, linear_x, angular_z):
        print(f'lin: {linear_x}, ang: {angular_z}')
        left_motor_speed = ((2*linear_x) - (0.51 * angular_z)) / 2
        right_motor_speed = ((2*linear_x) + (0.51*angular_z)) / 2
        print(f'Input Left: {left_motor_speed}, Right: {right_motor_speed}')
        
        left_motor_speed, right_motor_speed = self.scaling(left_motor_speed, right_motor_speed, 1.2)

        print(f'   Output Left: {left_motor_speed}, Right: {right_motor_speed}\n')
    
        left.move(left_motor_speed)
        right.move(right_motor_speed)
    
    def scaling(self, left, right, target):
        """
        Scales the left and right motor speeds to a target value while maintaining their ratio.

        Args:
            left (float): The speed of the left motor.
            right (float): The speed of the right motor.
            target (float): The target speed value.

        Returns:
            tuple: A tuple containing the scaled left and right motor speeds.
        """
        velThreshold  = 0.01
        if abs(left) < velThreshold:
            left = 0
        if abs(right) < velThreshold:
            right = 0
        l, r = 1, 1
        if left != 0:
           l = target / abs(left)
        if right != 0:
           r = target / abs(right)
        ratio = max(l, r)
        left = left * ratio
        right = right * ratio
        
        #if left * right < 0:
        #    if left==right:
        #        if left > 0:
        #            right=0
        #        if right>0:
        #            left=0        
        
        # clamp in between -1.4 to 1.4
        left = max(-(target*2), min(target*2, left))
        right = max(-(target*2), min(target*2, right))
    
        return left, right
        
def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()
