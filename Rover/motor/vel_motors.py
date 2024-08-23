# contributors:
#   Jacobus Burger (Jun 2024)
#   Jacobus Burger (Aug 2024)
# description:
#   This is a low level wrapper for the motors (the motor control board, DAC, and associated GPIO pins). It includes only basic functionality.
import gpiozero
import board
import busio
from adafruit_mcp4725 import MCP4725


class Motor():
    def __init__(self, DIR_PIN, SPEED_ADDR, I2C_INTERFACE, dir_reversed=True):
        # attach digital pins for controlling features
        self.dir_reversed = dir_reversed
        self.DIR = gpiozero.DigitalOutputDevice(DIR_PIN, initial_value=dir_reversed)
        # self.STOP = gpiozero.DigitalOutputDevice(STOP_PIN, active_high=False)      # 1 = off, 0 = on
        # connect I2C DAC for controlling speed
        while not I2C_INTERFACE.try_lock():
            pass
        try:
            devices = I2C_INTERFACE.scan()
            print("I2C devices found:", [hex(device) for device in devices])
        finally:
            I2C_INTERFACE.unlock()
        self.SPEED = MCP4725(I2C_INTERFACE, address=SPEED_ADDR)


    def enable(self):
        """enable motors with pin 13"""
        MOTOR_ENABLE.on()


    def disable(self):
        MOTOR_ENABLE.off()


    def move(self, velocity, forward=True):
        # print("move: ")
        # y = mx + b where y = velocity, m = ratio, and x is normalized voltage value
        # so to find x, x = (y-b)/m 
        # reselt from linear regression 
        # y = 5.2667x + 0.22

        # print(" min((max({}-0.22) / 5.2667, 1.0), 0.0)".format(velocity))
        normalized_voltage = max(0.0, min((velocity-0.22) / 5.2667, 1.0))
        
        # print("norm_volt = {}".format(normalized_voltage))
        # where 0.0 is 0v and 1.0 is 5v
        self.SPEED.normalized_value = normalized_voltage
        # self.SPEED.normalized_value = velocity # it is normalized voltage
        # set direction value (when dir_reversed then forward = 1 otherwise forward = 0)
        self.DIR.value = int(forward) if self.dir_reversed else int(not forward)
