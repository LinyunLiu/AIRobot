import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import gpiozero
from std_msgs.msg import Bool


class LeftWheel(Node):
    def __init__(self):
        super().__init__('left_wheel')
        self.get_logger().info('Left Wheel Listening')
        self.vel_L_pub = self.create_publisher(Float32, 'vel_L', 10)
        self.timer = self.create_timer(0.01, self.publish_vel)
        self.velocity = Float32()
        
        # to find the elapsed time since the last pulse
        self.prev_time = 0
        # pulse times to find rpm (note: measured in ns)
        self.pulse_time_u = 0
        self.pulse_time_v = 0
        self.pulse_time_w = 0
        self.pulse_time_avg = 0
        # direction
        self.direction = 1
        # pulse/revolution per minute calculations
        self.PREV_RPM = 0
        self.RPM = 0
        self.RPM_THRESHOLD = 1  # min RPM before being set to 0
        self.MOTOR_IDLE_TIMEOUT = 0.05  # 400 ms
        # LERP rate for smoothing RPM
        self.ALPHA = 0.3
                
        # to reference the U, V, and W pins
        self.U = gpiozero.DigitalInputDevice(23)
        self.V = gpiozero.DigitalInputDevice(24)
        self.W = gpiozero.DigitalInputDevice(25)

        self.U.when_activated = self.u 
        self.V.when_activated = self.v
        self.W.when_activated = self.w
        
        # Direction
        self.dirL_sub = self.create_subscription(
            Bool,
            'dir_L',
            self.listener_callback,
            10) 
        
        self.loop = self.create_timer(0.001,self.readHallSensor)
        
    def listener_callback(self, msg):
        self.direction = -1 if msg.data else 1
        
    # Created by Jacobus Burger (2024-08-01) with advice from Finian Lughtigheid
    # Linear intERPolation of RPM datapoints for smoothing (Thanks Finian!)
    def lerp(self, x0: float, x1: float, alpha: float) -> float:
        return (1 - alpha) * x0 + alpha * x1

    def publish_vel(self):
        self.velocity.data = self.direction * self.RPM/60 * 0.51867
        self.vel_L_pub.publish(self.velocity)
        # self.get_logger().info(f"Publishing: {str(self.velocity.data)}")
    
    def rpm(self, pulse_time):
        if pulse_time == 0:
            return 0
        Hz = 1 / pulse_time
        # https://www.eevblog.com/forum/projects/calculating-rpm-from-bemf-frequency-of-bldc-motor/
        # https://www.digikey.com/en/blog/using-bldc-hall-sensors-as-position-encoders-part-1
        # 2*(pulses_per_second / pulses_per_revolution) * seconds_per_minute
        # NOTE: There are 90 pulses in one revolution
        return 2 * (Hz / 90) * 60

    def setRPM(self):
        self.PREV_RPM = self.RPM
        self.RPM = self.lerp(self.PREV_RPM, self.rpm(self.pulse_time_avg), self.ALPHA)
    
    def u(self):
        # update rpm
        elapsed_time = time.time()
        self.pulse_time_u = elapsed_time - self.prev_time
        self.pulse_time_avg = (self.pulse_time_u + self.pulse_time_v + self.pulse_time_w) / 3
        self.PREV_RPM = self.RPM
        self.RPM = self.lerp(self.PREV_RPM, self.rpm(self.pulse_time_avg), self.ALPHA)
        self.prev_time = elapsed_time

    def v(self):
        # update rpm
        elapsed_time = time.time()
        self.pulse_time_v = elapsed_time - self.prev_time
        self.pulse_time_avg = (self.pulse_time_u + self.pulse_time_v + self.pulse_time_w) / 3
        self.PREV_RPM = self.RPM
        self.RPM = self.lerp(self.PREV_RPM, self.rpm(self.pulse_time_avg), self.ALPHA)
        self.prev_time = elapsed_time

    def w(self):
        # update rpm
        elapsed_time = time.time()
        self.pulse_time_w = elapsed_time - self.prev_time
        self.pulse_time_avg = (self.pulse_time_u + self.pulse_time_v + self.pulse_time_w) / 3
        self.PREV_RPM = self.RPM
        self.RPM = self.lerp(self.PREV_RPM, self.rpm(self.pulse_time_avg), self.ALPHA)
        self.prev_time = elapsed_time

    def readHallSensor(self):
        if self.RPM <= self.RPM_THRESHOLD:
            self.PREV_RPM = 0
            self.RPM = 0
        if time.time() - self.prev_time > self.MOTOR_IDLE_TIMEOUT:
            self.RPM = 0
        # publish vel
        self.publish_vel()
            
def main(args=None):
    rclpy.init(args=args)
    node = LeftWheel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    


