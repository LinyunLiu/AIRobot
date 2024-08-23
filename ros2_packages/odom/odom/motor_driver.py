# Oliver (LinYun) Liu, Jacobus Burger
# Final Driver Code for motor control system
from .motor_wrapper import *
from time import sleep
from threading import Thread

# from digitalio import DigitalInOut
# from adafruit_mcp4725 import MCP4725
# create I2C interface
i2c = busio.I2C(board.SCL, board.SDA)
# DIR, I2C_ADDR, I2C_INTERFACE, [dir_reversed]
left = Motor(19, 0x61, i2c, dir_reversed=False)
right = Motor(20, 0x60, i2c, dir_reversed=True)

# start watcher thread to disable motors when button is pressed
EMERGENCY_DISABLE = gpiozero.DigitalInputDevice(6)
def emergency():
    while True:
        if EMERGENCY_DISABLE.value == 1:
            disable()
t = Thread(target=emergency)
t.start()
print("ready")

# enable/disable motor power
MOTOR_ENABLE = gpiozero.DigitalOutputDevice(13)
def enable():
    MOTOR_ENABLE.on()
def disable():
    MOTOR_ENABLE.off()




# WARNING: speed is in m/s
DEFAULT_SPEED=0.70

enable()

def stop():
    left.move(0)
    right.move(0)
stop()


def forward(speed=DEFAULT_SPEED):
    stop()
    left.move(speed)
    right.move(speed)


def test(speed, sec):
    forward(speed)
    sleep(sec)
    stop()


def reverse(speed=DEFAULT_SPEED):
    stop()
    left.move(speed, forward=False)
    right.move(speed, forward=False)


def turn_right(speed=DEFAULT_SPEED, rate=0.3):
    left.move(speed+rate)
    right.move(speed)


def turn_left(speed=DEFAULT_SPEED, rate=0.3):
    left.move(speed)
    right.move(speed+rate)


def spin_right(speed=DEFAULT_SPEED):
    left.move(speed)
    right.move(speed, forward=False)


def spin_left(speed=DEFAULT_SPEED):
    left.move(speed, forward=False)
    right.move(speed)
