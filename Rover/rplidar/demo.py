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
        angle: angle in degrees given from LiDAR
        distance: distance in mm given from LiDAR
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


def measure(scan,ran):
    avg = [0,0,0] # F, L, R
    c = [0,0,0] # F, L, R
    
    for s in scan:
        front = detect(s[1], s[2], target=0, range=ran)
        left = detect(s[1], s[2], target=270, range=ran)
        right = detect(s[1], s[2], target=90, range=ran)
        
        if front == 1:
            avg[0] += s[2]
            c[0] += 1
        
        if left == 1:
            avg[1] += s[2]
            c[1] += 1
            
        if right == 1:
            avg[2] += s[2]
            c[2] += 1
    
    for i in range(3):
        if c[i] > 0:
            avg[i] /= c[i]
        
    return [int(a) for a in avg]
    
def lidarLoop(lidar:RPLidar, range=300,wide=10) -> None:
    """
    loop for for the lidar scan
    args
        lidar: LiDAR object
        TOGGLE_TIME: time to activate detection
    """
    
    try:
        for scan in lidar.iter_scans():
            print(measure(scan,range))
           
    except KeyboardInterrupt:
        print("STOP")
        lidar.stop()
        lidar.stop_motor()  
        sys.exit(0)

# lidarLoop(lidar, range=800, wide=10)

