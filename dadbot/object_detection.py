import os
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
import cv2
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge

from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference

class ObjectDetection(Node):
  def __init__(self):
    super().__init__('object_detection')
    self.subscription1 = Subscriber(self, Image, '/image_raw')
    self.subscription2 = Subscriber(self, Image, '/fisheye_image_raw')

    # Synchronizer for the image topics
    self.sync = ApproximateTimeSynchronizer([self.subscription1, self.subscription2], 10, 0.1)
    self.sync.registerCallback(self.image_callback)

    # Publisher for the combined image
    # self.image_pub = self.create_publisher(Image, 'combined_image_topic', 10)

    # Define a QoS profile with best_effort reliability
    qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        depth=5
    )

    self.publisher1 = self.create_publisher(CompressedImage, '/object_detection_image', qos_profile)
    self.publisher2 = self.create_publisher(CompressedImage, '/fisheye_object_detection_image', qos_profile)

    self.br = CvBridge()
    default_model_dir = '/home/mattsroufe/coral/pycoral/test_data/'
    default_model = 'ssd_mobilenet_v2_coco_quant_postprocess_edgetpu.tflite'
    default_labels = '/home/mattsroufe/coral/pycoral/test_data/coco_labels.txt'
    self.interpreter = make_interpreter(os.path.join(default_model_dir,default_model))
    self.interpreter.allocate_tensors()
    self.labels = read_label_file(default_labels)
    self.inference_size = input_size(self.interpreter)
    self.threshold = 0.1
    self.top_k = 3
    self.i = 0

  def image_callback(self, image_msg1, image_msg2):
      compressed1 = self.listener_callback(image_msg1)
      compressed2 = self.listener_callback(image_msg2)

      self.publisher1.publish(compressed1)
      self.publisher2.publish(compressed2)
      # self.publisher.publish(self.br.cv2_to_imgmsg(cv2_im, encoding='rgb8'))
      # print('received ' + str(self.i))
      # print(objs)
      self.i = self.i + 1
      # cv2.waitKey(1)

  def listener_callback(self, data):
      frame = self.br.imgmsg_to_cv2(data)

      # scale_percent = 50 # percent of original size
      # width = int(frame.shape[1] * scale_percent / 100)
      # height = int(frame.shape[0] * scale_percent / 100)
      # dim = (width, height)
      # resized = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)

      # cv2_im = resized
      cv2_im = frame
      cv2_im_rgb = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
      cv2_im_rgb = cv2.resize(cv2_im_rgb, self.inference_size)
      run_inference(self.interpreter, cv2_im_rgb.tobytes())
      objs = get_objects(self.interpreter, self.threshold)[:self.top_k]
      cv2_im = append_objs_to_img(cv2_im, self.inference_size, objs, self.labels)

      # new_image = self.br.cv2_to_imgmsg(cv2_im, encoding='rgb8')
      success, buffer = cv2.imencode('.jpg', cv2_im)

      if not success:
          self.get_logger().error('Failed to encode image')
          return

      msg = CompressedImage()
      msg.header.stamp = self.get_clock().now().to_msg()
      msg.format = 'jpeg'
      msg.data = np.array(buffer).tobytes()
      return msg

def main(args=None):
    rclpy.init(args=args)
    object_detection = ObjectDetection()
    rclpy.spin(object_detection)
    object_detection.destroy_node()
    rclpy.shutdown()

def append_objs_to_img(cv2_im, inference_size, objs, labels):
    height, width, channels = cv2_im.shape
    scale_x, scale_y = width / inference_size[0], height / inference_size[1]
    for obj in objs:
        bbox = obj.bbox.scale(scale_x, scale_y)
        x0, y0 = int(bbox.xmin), int(bbox.ymin)
        x1, y1 = int(bbox.xmax), int(bbox.ymax)

        percent = int(100 * obj.score)
        label = '{}% {}'.format(percent, labels.get(obj.id, obj.id))

        cv2_im = cv2.rectangle(cv2_im, (x0, y0), (x1, y1), (0, 255, 0), 2)
        cv2_im = cv2.putText(cv2_im, label, (x0, y0+30),
                             cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
    return cv2_im


if __name__ == '__main__':
  main()

