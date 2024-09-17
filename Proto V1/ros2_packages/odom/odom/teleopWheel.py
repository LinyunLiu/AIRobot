import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import getch

class teleopWheel(Node):
    def __init__(self):
        super().__init__('teleopWheel')
        self.get_logger().info('teleop Wheel Listening: Make sure Left and Right Wheels are Off')
        self.vel_L_pub = self.create_publisher(Float32, 'vel_L', 10)
        self.vel_R_pub = self.create_publisher(Float32, 'vel_R', 10)
        # self.timer = self.create_timer(0.1, self.publish_vel)
        self.msgL = Float32()
        self.msgR = Float32()
        self.vel = 0.0
        
    # def publish_vel(self):
    #     self.velocity.data = 0.01 # assign Value here
    #     self.vel_L_pub.publish(self.velocity)
    #     self.get_logger().info(f"Publishing: {str(self.velocity.data)}")
    
    def main_loop(self):
        while rclpy.ok():
            key = getch.getch()
            if ord(key) == 27: # ESC key
                self.get_logger().info("ESC: Terminate")
                break
            else:
                print(key)
                self.publish_cmd(key)
                # try:
                #     print("AA")
                # except:
                #     self.get_logger().error("Invalid Keyboard Input")
                #     break
                
    def publish_cmd(self, key):
        self.msgL = Float32()
        self.msgR = Float32()
        self.vel = 0.05
        match key:
            case "w":
                print('forward')
                self.msgL.data = self.vel
                self.msgR.data = self.vel
            case "a":
                print('left')
                self.msgL.data = self.vel 
                self.msgR.data = self.vel * 2
            case "s": 
                print('backward')
                self.msgL.data = -self.vel
                self.msgR.data = -self.vel
            case "d":
                print('right')
                self.msgL.data = self.vel 
                self.msgR.data = -self.vel 
            case " ":
                print('stop')
                self.msgL.data = 0.0
                self.msgR.data = 0.0
                
        self.vel_L_pub.publish(self.msgL)
        self.vel_R_pub.publish(self.msgR)
        self.get_logger().info(f'Publishing: "{str(self.msgL.data)}, {str(self.msgR.data)}"')
    
def main(args=None):
    rclpy.init()
    node = teleopWheel()
    try:
        node.main_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
