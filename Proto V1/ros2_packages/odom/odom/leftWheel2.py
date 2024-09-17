import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import gpiozero
from std_msgs.msg import Bool
from .taco import *

class LeftWheel(Node):
    def __init__(self):
        super().__init__('left_wheel')
        self.get_logger().info('Left Wheel Listening')
        self.vel_L_pub = self.create_publisher(Float32, 'vel_L', 10)
        self.timer = self.create_timer(0.001, self.publish_vel)
        self.velocity = Float32()
        
        self.left = WheelEncoder(23,24,25, 0.0085, alpha=0.3)

        # Direction
        self.direction = 1
        self.dirL_sub = self.create_subscription(
            Bool,
            'dir_L',
            self.listener_callback,
            10) 
        
        self.loop = self.create_timer(0.001,self.readHallSensor)
        
    def listener_callback(self, msg):
        self.direction = -1 if msg.data else 1
        
    def publish_vel(self):
        self.velocity.data = self.left.direction * (self.left.rpm / 60) * 0.51867 # m/s
        self.vel_L_pub.publish(self.velocity)
        
    def readHallSensor(self):
        if self.left.timed_out(0.05):
            self.left.rpm = 0 
            self.left.direction = 1
        # publish vel
        self.publish_vel()
            
def main(args=None):
    rclpy.init(args=args)
    node = LeftWheel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    


