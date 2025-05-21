import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class USBCameraPublisher(Node):
    def __init__(self):
        super().__init__('usb_camera_publisher')
        self.publisher = self.create_publisher(Image, '/usb_camera/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1/20.0, self.publish_frame)  # 20 FPS

        self.cap = cv2.VideoCapture(0)  # /dev/video0

        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara USB.")

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("No se pudo leer el frame de la cámara.")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = USBCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
