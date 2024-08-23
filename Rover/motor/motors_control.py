from controller import *
import threading
import os

def connect_to_wifi(ssid, password):
    command = f'nmcli device wifi connect {ssid} password {password}'
    os.system(command)
# connect_to_wifi("JP", "qwer1234")

def motor(command):
    if command == "w":
        print("DRIVE")
        drive()
    elif command == "s":
        print("REVERSE")
        reverse()
    elif command == "a":
        print("TURN LEFT")
        turn_left()
    elif command == "d":
        print("TURN RIGHT")
        turn_right()
    elif command == "q":
        print("SPIN LEFT")
        spin_left()
    elif command == "e":
        print("SPIN RIGHT")
        spin_right()
    elif command == 'f':
        print("FORWARD")
        forward()
    elif command == 'b':
        print("BACKWARD")
        backward()
    elif command == 'l':
        print("STEP LEFT")
        step_left()
    elif command == 'r':
        print("STEP RIGHT")
        step_right()
    elif command == "x":
        print("STOP")
        stop()
    else:
        print("STOP")
        stop()

# ================================ LIDAR =======================================
from rplidar import RPLidar
import sys
lidar = RPLidar('/dev/ttyUSB0', timeout=3)
lidar.stop()
lidar.stop_motor()
info = lidar.get_info()
health = lidar.get_health()
def detect(angle:float, distance:float, target:float, ran:float=1500, wide:float=150):
    left = ((target - wide)%360 - 180)%360
    right = ((target + wide)%360 - 180)%360
    angle = (angle - 180)%360
    
    if left <= angle <= right:
        if distance < ran:
            return 1
    else:
        return 2
    return 0
def measure(scan,ran, wide=10):
    avg = [0,0,0]# F, L, R
    c = [0,0,0] # F, L, R   
    for s in scan:
        front = detect(s[1], s[2], target=0, ran=ran, wide=wide)
        left = detect(s[1], s[2], target=270, ran=ran, wide=wide)
        right = detect(s[1], s[2], target=90, ran=ran, wide=wide)       
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
def lidarLoop(lidar:RPLidar, ran=600, wide=10) -> None:
    try:
        for scan in lidar.iter_scans():
            avg = measure(scan,ran,wide=wide)
            if (avg[0] != 0) and (avg[0] < ran):
                print("TOO CLOSE ->", avg[0])
                stop()
            else:
                pass
                # print("GOOD ->", avg[0])                      
    except KeyboardInterrupt:
        lidar.stop()
        lidar.stop_motor()  
        sys.exit(0)
Lidar = threading.Thread(target=lidarLoop, args=[lidar],kwargs={'ran':600, 'wide':150})
#Lidar.start()
# ================================= END =======================================    


# ======================= START LISTENING TO COMMAND ==================
import socket
server_host = '192.168.14.152'  
server_port = 12345
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((server_host, server_port))
server_socket.listen(1)
print(f"Listening on {server_host}:{server_port}...")
client_socket, client_address = server_socket.accept()
print(f"Accepted connection from {client_address}")
try:
    while True:
        data = client_socket.recv(1024).decode('utf-8')
        if not data:
            stop()
            break
        motor(data)
except KeyboardInterrupt:
    stop()
finally:
    stop()
    print("Closing connection...")
    client_socket.close()
    server_socket.close()
    Lidar.join()
