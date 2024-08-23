# this file is an abstraction to control the movement system on a high level
from motors import *
from time import sleep
# from digitalio import DigitalInOut
# from adafruit_mcp4725 import MCP4725
# create I2C interface
i2c = busio.I2C(board.SCL, board.SDA)
# DIR, I2C_ADDR, I2C_INTERFACE, [dir_reversed]
left = Motor(19, 0x61, i2c, dir_reversed=False)
right = Motor(20, 0x60, i2c, dir_reversed=True)

SPEED = 10

def stop():
    left.move(0)
    right.move(0)
stop()

def drive(speed):
    left.move(speed)
    right.move(speed)

def reverse(speed):
    left.move(speed, forward=False)
    right.move(speed, forward=False)

def turn_right(speed, right=4):
    left.move(speed+right)
    right.move(speed)

def turn_left(speed, left):
    left.move(speed)
    right.move(speed+left)


def spin_left():
    left.move(0)
    right.move(SPEED)
def spin_right():
    left.move(SPEED)
    right.move(0)


def forward(time=1.5):
    left.move(SPEED)
    right.move(SPEED)
    sleep(time)
    stop()
def backward(time=1.5):
    left.move(SPEED, forward=False)
    right.move(SPEED, forward=False)
    sleep(time)
    stop()
def step_left(time=1.5):
    left.move(0)
    right.move(SPEED)
    sleep(time)
    stop()
def step_right(time=1.5):
    left.move(SPEED)
    right.move(0)
    sleep(time)
    stop()

