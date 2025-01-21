import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import asyncio
import websockets
from asyncio import Lock

class MultiClientVideoServer(Node):
    def __init__(self):
        super().__init__('multi_client_video_server')
        self.bridge = CvBridge()
        self.server_port = 8765  # Change to your desired port
        self.image_publishers = {}  # Thread-safe structure to store publishers for each client IP
        self.publisher_lock = Lock()  # Async lock to protect access to image_publishers

    async def handle_client(self, websocket, path):
        client_ip = websocket.remote_address[0]
        self.get_logger().info(f"New connection from {client_ip}")

        async with self.publisher_lock:
            if client_ip not in self.image_publishers:
                topic_name = f"/video_stream/{client_ip.replace('.', '_')}"
                self.image_publishers[client_ip] = self.create_publisher(Image, topic_name, 10)
                self.get_logger().info(f"Created publisher for topic: {topic_name}")

        try:
            async for message in websocket:
                # Decode the received bytes to a NumPy array
                np_arr = np.frombuffer(message, dtype=np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if frame is not None:
                    # Convert frame to a ROS Image message and publish it
                    ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    async with self.publisher_lock:
                        self.image_publishers[client_ip].publish(ros_image)
                else:
                    self.get_logger().warning(f"Failed to decode frame from {client_ip}")
        except Exception as e:
            self.get_logger().error(f"Error handling client {client_ip}: {e}")
        finally:
            self.get_logger().info(f"Connection closed for {client_ip}")

    async def start_server(self):
        async with websockets.serve(self.handle_client, "0.0.0.0", self.server_port):
            self.get_logger().info(f"WebSocket server started on port {self.server_port}")
            await asyncio.Future()  # Run forever

    def run(self):
        loop = asyncio.get_event_loop()
        try:
            loop.run_until_complete(self.start_server())
        finally:
            loop.close()


def main(args=None):
    rclpy.init(args=args)
    video_server = MultiClientVideoServer()

    try:
        video_server.run()
    except KeyboardInterrupt:
        video_server.get_logger().info('Shutting down server...')
    finally:
        video_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
