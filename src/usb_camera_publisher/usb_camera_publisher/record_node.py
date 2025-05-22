import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class RecordAnnotated(Node):
    def __init__(self):
        super().__init__('record_node')
        self.bridge = CvBridge()

        # Obtener fecha actual
        date_str = datetime.now().strftime('%Y_%m_%d')

        # Crear carpeta de salida si no existe
        self.output_dir = os.path.expanduser(
            '~/ros2_ws/src/usb_camera_publisher/usb_camera_publisher/videos/Videos_annotated')
        os.makedirs(self.output_dir, exist_ok=True)

        # Nombres de archivo con fecha
        self.front_output_path = os.path.join(self.output_dir, f'{date_str}_front_annotated.mp4')
        self.rear_output_path = os.path.join(self.output_dir, f'{date_str}_rear_annotated.mp4')

        self.front_writer = None
        self.rear_writer = None

        # Suscripciones a los tópicos anotados
        self.front_sub = self.create_subscription(
            Image, '/camera/front/image_annotated', self.front_callback, 10)

        self.rear_sub = self.create_subscription(
            Image, '/camera/rear/image_annotated', self.rear_callback, 10)

        self.get_logger().info('Record node started. Waiting for annotated image data...')

    def front_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.front_writer is None:
            height, width = frame.shape[:2]
            self.front_writer = cv2.VideoWriter(
                self.front_output_path, cv2.VideoWriter_fourcc(*'mp4v'), 20, (width, height)
            )
            self.get_logger().info(f"▶️ Grabando imagen anotada frontal en: {self.front_output_path}")

        self.front_writer.write(frame)

    def rear_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.rear_writer is None:
            height, width = frame.shape[:2]
            self.rear_writer = cv2.VideoWriter(
                self.rear_output_path, cv2.VideoWriter_fourcc(*'mp4v'), 20, (width, height)
            )
            self.get_logger().info(f"▶️ Grabando imagen anotada trasera en: {self.rear_output_path}")

        self.rear_writer.write(frame)

    def destroy_node(self):
        if self.front_writer:
            self.front_writer.release()
            self.get_logger().info("✅ Grabación frontal finalizada y archivo guardado.")
        if self.rear_writer:
            self.rear_writer.release()
            self.get_logger().info("✅ Grabación trasera finalizada y archivo guardado.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RecordAnnotated()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
