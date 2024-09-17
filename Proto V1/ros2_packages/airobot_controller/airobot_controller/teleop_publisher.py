import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import getch
from std_msgs.msg import Bool

class TeleopPublisher(Node):
    def __init__(self):
        super().__init__('teleop_publisher')
        self.publisher_ = self.create_publisher(String, 'move_cmd', 10)        
        self.get_logger().info('Keyboard Input Listening...')

    def main_loop(self):
        while rclpy.ok():
            key = getch.getch()
            if ord(key) == 27: # ESC key
                self.get_logger().info("ESC: Terminate")
                break
            else:
                try:
                    self.publish_cmd(key)
                except:
                    self.get_logger().error("Invalid Keyboard Input")
                    break
                
    def publish_cmd(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

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
