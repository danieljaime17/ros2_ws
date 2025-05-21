import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class CameraRearNode(Node):
    def __init__(self):
        super().__init__('camera_rear')
        self.publisher = self.create_publisher(Image, '/camera/rear/image_raw', 10)
        self.bridge = CvBridge()

        video_path = os.path.expanduser('~/ros2_ws/src/usb_camera_publisher/usb_camera_publisher/videos/video_camera_rear.mp4')
        self.cap = cv2.VideoCapture(video_path)

        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir el video de la cámara trasera.")
            rclpy.shutdown()
            return

        self.timer = self.create_timer(1/20.0, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("El video ha terminado. Finalizando nodo.")
            self.timer.cancel()
            self.cap.release()
            rclpy.shutdown()
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraRearNode()
    rclpy.spin(node)
    if node is not None:
        node.destroy_node()
    rclpy.shutdown()

