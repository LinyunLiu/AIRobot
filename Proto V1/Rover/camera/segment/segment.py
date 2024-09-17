import cv2
from ultralytics import YOLO

# model = YOLO("yolov8n-seg.pt")
model = YOLO("/home/airobot/Rover/camera/segment/v2.pt")

video_path = "video.MP4"
cap = cv2.VideoCapture(video_path)

while cap.isOpened():
    success, frame = cap.read()

    if success:
        results = model(frame)

        annotated_frame = results[0].plot()

        cv2.imshow("YOLO", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    else:
        break

cap.release()
cv2.destroyAllWindows()