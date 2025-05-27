#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import PySpin
import cv2
import numpy as np

class CameraEthernetNode(Node):
    def __init__(self):
        super().__init__('camera_ethernet_node')
        self.publisher = self.create_publisher(Image, '/camera_ethernet/image_raw', 10)
        self.bridge = CvBridge()

        self.get_logger().info("Inicializando cámara Ethernet con PySpin...")

        self.system = PySpin.System.GetInstance()
        self.cam_list = self.system.GetCameras()

        if self.cam_list.GetSize() == 0:
            self.get_logger().error("❌ No se encontró ninguna cámara Ethernet conectada.")
            exit(1)

        self.cam = self.cam_list.GetByIndex(0)
        self.cam.Init()
        self.cam.BeginAcquisition()
        self.get_logger().info("✅ Cámara inicializada y en adquisición")

        self.timer = self.create_timer(1.0 / 20.0, self.capture_and_publish)

    def capture_and_publish(self):
        try:
            image_result = self.cam.GetNextImage(1000)  # Timeout de 1 segundo
            if image_result.IsIncomplete():
                self.get_logger().warning("Imagen incompleta")
                return

            image_data = image_result.GetNDArray()
            image_result.Release()

            # Aseguramos que tenga 3 canales si es necesario
            if len(image_data.shape) == 2:
                image_data = cv2.cvtColor(image_data, cv2.COLOR_GRAY2BGR)

            msg = self.bridge.cv2_to_imgmsg(image_data, encoding='bgr8')
            self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error al capturar o publicar imagen: {e}")

    def destroy_node(self):
        self.get_logger().info("⏹ Finalizando nodo y liberando cámara...")
        self.cam.EndAcquisition()
        self.cam.DeInit()
        self.cam_list.Clear()
        self.system.ReleaseInstance()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraEthernetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
