import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np

class ImagePublisher(Node):
  def __init__(self):
    super().__init__('image_publisher')

    global scale_percent

    # Declare parameters
    self.declare_parameter('video_device', 0)
    self.declare_parameter('frame_rate', 30)
    self.declare_parameter('scale_percent', 50)

    # Retrieve parameters
    video_device = self.get_parameter('video_device').get_parameter_value().integer_value
    frame_rate = self.get_parameter('frame_rate').get_parameter_value().integer_value
    scale_percent = self.get_parameter('scale_percent').get_parameter_value().integer_value

    self.get_logger().info(f'video_device: {video_device}')
    self.get_logger().info(f'frame_rate: {frame_rate}')

    self.publisher_ = self.create_publisher(CompressedImage, 'image_raw/compressed', 10)
    fps = frame_rate
    timer_period = 1/fps
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.cap = cv2.VideoCapture(video_device)
    self.br = CvBridge()

  def timer_callback(self):
    ret, frame = self.cap.read()

    if ret == True:
        width = int(frame.shape[1] * scale_percent / 100)
        height = int(frame.shape[0] * scale_percent / 100)
        dim = (width, height)
        resized = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)
        success, buffer = cv2.imencode('.jpg', resized)

        if not success:
            self.get_logger().error('Failed to encode image')
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data = np.array(buffer).tobytes()

        self.publisher_.publish(msg)
        # self.publisher_.publish(self.br.cv2_to_imgmsg(resized))

def main(args=None):
  rclpy.init(args=args)
  image_publisher = ImagePublisher()
  rclpy.spin(image_publisher)
  image_publisher.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
