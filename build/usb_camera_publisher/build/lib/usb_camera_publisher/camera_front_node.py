import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class CameraFrontNode(Node):
    def __init__(self):
        super().__init__('camera_front')
        self.publisher = self.create_publisher(Image, '/camera/front/image_raw', 10)
        self.bridge = CvBridge()

        video_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'videos/video_camera_front.mp4')
        self.cap = cv2.VideoCapture(video_path)

        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir el video de la cámara frontal.")

        self.timer = self.create_timer(1/20.0, self.publish_frame)  # 20 FPS

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Reinicia el video
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraFrontNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

