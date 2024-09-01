import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np

class ImageViewerNode(Node):
    def __init__(self):
        super().__init__('image_viewer_node')
        
        # Create a CvBridge instance
        self.bridge = CvBridge()
        
        # Define QoS profile (if necessary)
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,  # or RELIABLE
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create a subscription to the image topic (compressed)
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',  # Adjust to your topic name
            self.image_callback,
            qos_profile
        )

    def image_callback(self, msg):
        # Convert the ROS Image message to an OpenCV image
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # Display the image using OpenCV
        cv2.imshow("Image Viewer", image)
        
        # Add a delay to keep the window open (1ms)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageViewerNode()
    rclpy.spin(node)
    
    # Clean up
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

