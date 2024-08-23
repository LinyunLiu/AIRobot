import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .controller import *
from std_msgs.msg import Bool

class MotorSubscriber(Node):
    def __init__(self):
        super().__init__('motor_subscriber')
        self.subscription = self.create_subscription(
            String,
            'move_cmd',
            self.listener_callback,
            10) 
        # self.timer = self.create_timer(1.0, self.check_publisher)
        
        self.l_dir_pub = self.create_publisher(Bool, 'dir_L', 10)
        self.r_dir_pub = self.create_publisher(Bool, 'dir_R', 10)
        self.timerL = self.create_timer(0.001, self.pub_L_dir)
        self.timerR = self.create_timer(0.001, self.pub_R_dir)

    def listener_callback(self, msg):
        cmd = msg.data
        self.motor(cmd)
        
    def check_publisher(self):
        """Check if there is at least one publisher. If there is none, keep printing 'No Publisher'"""
        publisher_count = self.count_publishers('move_cmd')
        if publisher_count == 0:
            print("No Pulisher")
    
    def pub_L_dir(self):
        dir = Bool()
        dir.data = bool(getLeftDir())
        self.l_dir_pub.publish(dir)
        
    def pub_R_dir(self):
        dir = Bool()
        dir.data = bool(getRightDir())
        self.r_dir_pub.publish(dir)

    def motor(self, command):
        if command == "w":
            # print("DRIVE")
            drive()
        elif command == "s":
            # print("REVERSE")
            reverse()
        elif command == "a":
            # print("TURN LEFT")
            turn_left()
        elif command == "d":
            # print("TURN RIGHT")
            turn_right()
        elif command == "q":
            # print("SPIN LEFT")
            spin_left()
        elif command == "e":
            # print("SPIN RIGHT")
            spin_right()
        elif command == 'f':
            # print("FORWARD")
            forward()
        elif command == 'b':
            # print("BACKWARD")
            backward()
        elif command == 'l':
            # print("STEP LEFT")
            step_left()
        elif command == 'r':
            # print("STEP RIGHT")
            step_right()
        elif command == "x":
            # print("STOP")
            stop()
        else:
            # print("STOP")
            stop()
        
def main(args=None):
    rclpy.init(args=args)
    node = MotorSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
