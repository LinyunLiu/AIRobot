# Basic ROS 2 program to publish real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
# Modified By:
# - Jun Park
  
# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from picamera2 import Picamera2
 
class CameraPublisher(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('camera_publisher')
      
    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
      
    # We will publish a message every 0.1 seconds
    timer_period = 0.5  # seconds # about 30FPS: 0.03 60FPS 0.167
      
    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)
         
    # Camera Object
    self.cap = Picamera2()
    # self.cap.preview_configuration.main.size = (1280, 720)
    self.cap.preview_configuration.main.format = "RGB888"
    self.cap.preview_configuration.align()
    self.cap.configure("preview")
    self.cap.start()
         
    # Used to convert between ROS and OpenCV images
    self.bridge = CvBridge()
   
  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.1 seconds.
    """
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    frame = self.cap.capture_array()

    # Convert numpy array to ROS Image message
    image_message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

    # Publish the image message
    self.publisher_.publish(image_message)
    self.get_logger().info('Publishing image')
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_publisher = CameraPublisher()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_publisher)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_publisher.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()