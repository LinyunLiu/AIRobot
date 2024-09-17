# Linyun Liu (2024)
import cv2
from picamera2 import Picamera2
from ultralytics import YOLO
import sys
import os
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../motor')))
import controller

# Initialize the Picamera2
picam2 = Picamera2()
picam2.preview_configuration.main.size = (1280, 720)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# Load the YOLOv8 model
model = YOLO("/home/airobot/Rover/camera/segment/v2.pt")



def extract(results, frame):
    boxes = results.boxes.xywh.cpu()
    classes = results.boxes.cls.cpu().tolist() # return detected objects ID -> List of int
    names = results.names # return all possible names for dtection -> Dictionary (int: name)
    confs = results.boxes.conf.float().cpu().tolist() # return detected objects confidence score -> List of Float
    output = []
    for i in range(len(classes)):
        result = []
        result.append(names[classes[i]])
        result.append(confs[i])

        x1, y1, x2, y2 = (results[i].boxes.xyxy)[0]
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        center_x = (x1+x2)//2
        center_y = (y1+y2)//2
        result.append([center_x, center_y])

        # cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

        output.append(result)

    return output # 2D array, i.e. [[person, 0.92, coordinates], [cat, 0.67, coordinates]], [detcted object, confidence score, object bounding box coordinates]

controller.SPEED = 10
controller.drive()

start_time = time.time()
try:  
    while True:
        elapsed_time = time.time() - start_time
        if elapsed_time >= 30:
            controller.stop()
            break

        frame = picam2.capture_array()

        # results_c = model_custom(frame)
        results = model(frame)
        os.system("clear")
        annotated_frame = results[0].plot()
        try:
            # result_c = extract(results_c[0])
            result_d = extract(results[0], annotated_frame)
            
            for r in result_d:
                print(f"{r[0]}: {r[1]} -> {r[2]}")

                x =  r[2][0]
                offset = 10
                # Center of the screen is x=480, y=360
                if x > 640+offset:
                    controller.adjust_to_right()
                    print("GOING RIGHT")
                if x < 640-offset:
                    controller.adjust_to_left()
                    print("GOING LEFT")
                else:
                    controller.drive()

                break


        except:
            pass
        #cv2.imshow("Camera", annotated_frame)
        if cv2.waitKey(1) == ord("q"):
            controller.stop()
            break
        # time.sleep_ms(10)
        
    cv2.destroyAllWindows()
except KeyboardInterrupt:
    os.system("clear")
    controller.stop()