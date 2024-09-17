# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
# Modified By:
# - Jun Park
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from ultralytics import YOLO
 
class CameraSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('camera_subscriber')
    self.get_logger().info('Camera Subscriber Start')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      '/image_raw', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
      
    # Used to convert between ROS and OpenCV images
    self.bridge = CvBridge()
    # self.model_default = YOLO("/home/airobot/Rover/camera/detection/yolov8n.pt")
    # self.model_default.export(format="ncnn")
    self.model_default = YOLO("/home/airobot/Rover/camera/detection/yolov8n_ncnn_model")
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.bridge.imgmsg_to_cv2(data)
    
    self.detect(current_frame)
    # annotated_frame=self.detect(current_frame)[0].plot()

    # # # Display image
    # cv2.imshow("camera", annotated_frame)
    
    # cv2.waitKey(1)
  
  def detect(self, frame):
    results_d = self.model_default(frame)

    result_d = self.extract(results_d[0])
    for r in result_d:
      result_str = f"{r[0]}: {r[1]} -> {r[2]} {r[3]}"
      self.get_logger().info(result_str)
      # if r[0] == 'person' and r[1] >= 0.6:
      #     print("person detected")
    return results_d
  
  def extract(self, results):
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
        # cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

        x, y, w, h = (results[i].boxes.xywh)[0]
        result.append(int(w)*int(h))

        output.append(result)

    return output # 2D array, i.e. [[person, 0.92, coordinates, area], [cat, 0.67, coordinates, area]], [detcted object, confidence score, object bounding box coordinates, box area]

def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = CameraSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()# Basic ROS 2 program to subscribe to real-time streaming 
