# this file is an abstraction to control the movement system on a high level
# from .motors import *
from .motors import *
from time import sleep
# from digitalio import DigitalInOut
# from adafruit_mcp4725 import MCP4725
# create I2C interface
i2c = busio.I2C(board.SCL, board.SDA)
# DIR, I2C_ADDR, I2C_INTERFACE, [dir_reversed]
left = Motor(19, 0x61, i2c, dir_reversed=False)
right = Motor(20, 0x60, i2c, dir_reversed=True)

SPEED = 40

def stop():
    left.move(0)
    right.move(0)
stop()

def set_speed(speed):
    SPEED = speed

# Keeps driving foward until stop() function is called
def drive():
    left.move(SPEED)
    right.move(SPEED)
# Keeps driving backward until stop() function is called
def reverse():
    left.move(SPEED, forward=False)
    right.move(SPEED, forward=False)

# Turn right with the left wheel speed 4+ (default)
def turn_right(speed=4):
    left.move(SPEED+speed)
    right.move(SPEED)
# Turn left with the right wheel speed 4+ (default)
def turn_left(speed=4):
    left.move(SPEED)
    right.move(SPEED+speed)

# keep spinning left until other action is called
def spin_left():
    left.move(0)
    right.move(SPEED)
# keep spinning right until other action is called
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

def getLeftDir():
    return left.DIR.value

def getRightDir():
    return not right.DIR.value
     
