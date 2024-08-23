import taco
import time

r = taco.WheelEncoder(17,27,22,taco.RADIUS)
l = taco.WheelEncoder(23,24,25,taco.RADIUS)

while 1:
    print(f"Right: {r.rpm_to_rads(r.rpm)*r.radius} m/s")
    print(f"Left: {l.rpm_to_rads(l.rpm)*l.radius} m/s")
    print()
    time.sleep(0.1)
    