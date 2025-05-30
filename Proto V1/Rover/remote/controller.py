# Oliver (LinYun) Liu, Jacobus Burger
# this file is an abstraction to control the movement system on a high level
from motors import *
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


# enable/disable motor power
MOTOR_ENABLE = gpiozero.DigitalOutputDevice(13)
def enable():
    MOTOR_ENABLE.on()
def disable():
    MOTOR_ENABLE.off()


# WARNING: speed is in m/s
DEFAULT_SPEED=0.60
FORWARD = True

enable()

def stop():
    left.move(0)
    right.move(0)
stop()


def forward(speed=DEFAULT_SPEED):
    global FORWARD
    stop()
    left.move(speed)
    right.move(speed)
    FORWARD = True


def reverse(speed=DEFAULT_SPEED):
    global FORWARD
    stop()
    left.move(speed, forward=False)
    right.move(speed, forward=False)
    FORWARD = False


def turn_right(speed=DEFAULT_SPEED, rate=0.3):
    global FORWARD
    if FORWARD == True:
        left.move(speed+rate)
        right.move(speed)
    else:
        left.move(speed+rate, forward=False)
        right.move(speed, forward=False)


def turn_left(speed=DEFAULT_SPEED, rate=0.3):
    global FORWARD
    if FORWARD == True:
        left.move(speed)
        right.move(speed+rate)
    else:
        left.move(speed, forward=False)
        right.move(speed+rate, forward=False)


def spin_right(speed=DEFAULT_SPEED):
    left.move(speed)
    right.move(0)


def spin_left(speed=DEFAULT_SPEED):
    left.move(0)
    right.move(speed)