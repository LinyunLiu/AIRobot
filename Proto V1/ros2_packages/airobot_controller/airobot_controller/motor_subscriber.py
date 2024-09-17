import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MotorSubscriber(Node):
    def __init__(self):
        super().__init__('motor_subscriber')
        self.subscription = self.create_subscription(
            String,
            'move_cmd',
            self.listener_callback,
            10) 
        self.timer = self.create_timer(1.0, self.check_publisher)

    def listener_callback(self, msg):
        print(str(msg))
    
    def check_publisher(self):
        """Check if there is at least one publisher. If there is none, keep printing 'No Publisher'"""
        publisher_count = self.count_publishers('move_cmd')
        if publisher_count == 0:
            print("No Pulisher")

def main(args=None):
    rclpy.init(args=args)
    node = MotorSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
