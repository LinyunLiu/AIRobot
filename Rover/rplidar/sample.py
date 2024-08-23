from rplidar import RPLidar
lidar = RPLidar('/dev/ttyUSB0', timeout=3)
lidar.stop()
lidar.stop_motor()

info = lidar.get_info()
# print(info)
health = lidar.get_health()
# print(health)

for measure in lidar.iter_measures():
    if len(measure) == 4:
        new_scan, quality, angle, distance = measure

        if 0< distance < 300:
            print((angle, distance))
    else:
        print(len(measure))

lidar.stop()
lidar.stop_motor()
lidar.disconnect()
