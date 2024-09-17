import rclpy
from rclpy.node import Node
import getch
from geometry_msgs.msg import Twist


class TeleopPublisher(Node):
    def __init__(self):
        super().__init__('teleop_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)        
        self.get_logger().info('Keyboard Input Listening...')

    def main_loop(self):
        speed = 0.8
        twist = Twist()
        while rclpy.ok():
            key = getch.getch()

            if key == 'w':
                twist.linear.x = speed
                twist.angular.z = 0.0
            elif key == 's':
                twist.linear.x = -speed
                twist.angular.z = 0.0
            elif key == 'a':
                twist.linear.x = 0.0
                twist.angular.z = 3.
            elif key == 'd':
                twist.linear.x = 0.0
                twist.angular.z = -3.
            elif key == 'q':
                self.get_logger().info('Quitting...')
                break
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                
            self.publisher_.publish(twist)
            self.get_logger().info(f'Publishing: linear_x = {twist.linear.x}, angular_z = {twist.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    node = TeleopPublisher()
    try:
        node.main_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
