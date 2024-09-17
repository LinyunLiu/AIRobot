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
    left = ((target - wide)%360 - 180)%360
    right = ((target + wide)%360 - 180)%360
    angle = (angle - 180)%360
    
    if left <= angle <= right:
        if distance < range:
            print(left,right)
            return 1
    else:
        return 2
    return 0

toggle = False
t_on = .0
spend_on = .0

t_off = .0
spend_off = .0

for new_scan, quality, angle, distance in lidar.iter_measures():
    result = detect(angle,distance,target=0,range=300,wide=10)
    if result == 0:
        if not toggle:
            spend_off += timer() - t_off
        else:
            toggle = False
            spend_off = .0
            t_off = timer()

        if spend_off > 1:
            led.off()
            toggle = False
            t_on = .0
            spend_on = .0

    elif result == 1:
        if toggle:
            spend_on += timer() - t_off
        else:
            toggle = True
            spend_on = .0
            t_on = timer()

        if spend_on > 1:
            led.on()
            toggle = True
            t_off = .0
            spend_off = .0

lidar.stop()
lidar.stop_motor()
lidar.disconnect()
