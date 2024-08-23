import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImagePublisher(Node):
  def __init__(self):
    super().__init__('image_publisher')
    self.publisher_ = self.create_publisher(CompressedImage, 'image_raw/compressed', 10)
    fps = 30
    timer_period = 1/fps
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.cap = cv2.VideoCapture(0)
    self.br = CvBridge()

  def timer_callback(self):
    ret, frame = self.cap.read()

    if ret == True:
        scale_percent = 50 # percent of original size
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
