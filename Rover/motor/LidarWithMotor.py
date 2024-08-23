from rplidar import RPLidar
from timeit import default_timer as timer
import gpiozero
import sys
# import controller
import time
     
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
        angle: angle in degrees given from LiDar
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
            return 1
    else:
        return 2
    return 0

def lidarLoop(lidar:RPLidar, OnDetect, OnNotDetect, TOGGLE_TIME:float = 0.5,
              target=0,range=300,wide=10)->None:
    """
    loop for for the lidar scan
    args
        lidar: LiDAR object
        OnDetect: function that will be called when LiDAR detects something
        OnNotDetect: function that will be called when LiDAR does not detect anything
        TOGGLE_TIME: time to activate detection
    """
    
    toggle = False
    t_on = .0
    spend_Stop = .0

    t_off = .0
    spend_Go = .0
    try:
        for new_scan, quality, angle, distance in lidar.iter_measures():
            # Change detect args if we need to change
            result = detect(angle,distance,target=target,range=range,wide=wide)
            if result == 0: # not detected
                if not toggle:
                    spend_Go += timer() - t_off
                else:
                    toggle = False
                    spend_Go = .0
                    t_off = timer()

                if spend_Go > TOGGLE_TIME:
                    OnNotDetect()
                    toggle = False
                    t_on = .0
                    spend_Stop = .0

            elif result == 1: # detected
                # print(angle,distance)
                if toggle:
                    spend_Stop += timer() - t_off
                else:
                    toggle = True
                    spend_Stop = .0
                    t_on = timer()

                if spend_Stop > TOGGLE_TIME:                
                    OnDetect()
                    toggle = True
                    t_off = .0
                    spend_Go = .0
                    
    except KeyboardInterrupt:
        print("STOP")
        lidar.stop()
        lidar.stop_motor()  
        # controller.motor.move(0)
        sys.exit(0)

def onDetect():
    print("DE")
    # controller.motor.move(0)

def onNotDetect():
    print("GO")
    # controller.motor.move(100)

lidarLoop(lidar, onDetect,onNotDetect, TOGGLE_TIME = 0.5, target=0 , range=700,wide=150)

