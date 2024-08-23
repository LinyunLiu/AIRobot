"""
This code demonstrates why we need timer to turn on and off the led.
If the object is on the edge of the target distance, the led toggles on and off rapidly,
which leads wrong detection
"""


from rplidar import RPLidar
from timeit import default_timer as timer
import gpiozero

led = gpiozero.LED(16)
     
lidar = RPLidar('/dev/ttyUSB0', timeout=3)
lidar.stop()
lidar.stop_motor()

info = lidar.get_info()
print(info)
health = lidar.get_health()
print(health)

def detect(angle:float, distance:float, target:float, range:float=1000, wide:float=10):
    """
    check if there is an object at target degree in range and distance
    args
        angle: angle in degrees
        distance: distance in mm
        target: target angle in degree
        range: maximum detecing distance in mm
        wide: from target angle left and rignt
    return
        int: 0=not detected, 1=detected, 2=not in angle
    """
    left = target - wide - 180
    right = target + wide - 180
    angle -= 180
    if left <= angle <= right:
        if distance < range:
            return 1
    else:
        return 2
    return 0

for new_scan, quality, angle, distance in lidar.iter_measures():
    # new_scan, quality, angle, distance = measure
    result = detect(angle,distance,target=0,range=300,wide=10)
    if result == 0:
        led.off()
            
    elif result == 1:
        led.on()
            
lidar.stop()
lidar.stop_motor()
lidar.disconnect()
