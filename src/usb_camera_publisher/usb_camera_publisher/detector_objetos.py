import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class DetectorObjetos(Node):
    def __init__(self):
        super().__init__('detector_objetos')

        self.bridge = CvBridge()

        # Cargar modelo de detección
        model_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'models/')
        model_weights = os.path.join(model_path, 'frozen_inference_graph.pb')
        model_config = os.path.join(model_path, 'ssd_mobilenet_v1_coco.pbtxt')
        self.net = cv2.dnn.readNetFromTensorflow(model_weights, model_config)

        # Diccionario de etiquetas (ejemplo reducido)
        self.labels = {
            1: 'person', 2: 'bicycle', 3: 'car', 4: 'motorcycle', 5: 'airplane', 6: 'bus',
            7: 'train', 8: 'truck', 9: 'boat', 17: 'cat', 18: 'dog', 77: 'cell phone'
        }

        # Suscripciones
        self.create_subscription(Image, '/usb_camera/image_raw', self.callback_usb, 10)
        self.create_subscription(Image, '/camera/front/image_raw', self.callback_front, 10)
        self.create_subscription(Image, '/camera/rear/image_raw', self.callback_rear, 10)

        # Publicadores
        self.pub_usb = self.create_publisher(Image, '/usb_camera/image_annotated', 10)
        self.pub_front = self.create_publisher(Image, '/camera/front/image_annotated', 10)
        self.pub_rear = self.create_publisher(Image, '/camera/rear/image_annotated', 10)

    def procesar_imagen(self, frame):
        (h, w) = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, size=(300, 300), swapRB=True, crop=False)
        self.net.setInput(blob)
        detections = self.net.forward()

        for i in range(detections.shape[2]):
            confidence = float(detections[0, 0, i, 2])
            if confidence > 0.5:
                class_id = int(detections[0, 0, i, 1])
                label = self.labels.get(class_id, f'class {class_id}')
                box = detections[0, 0, i, 3:7] * [w, h, w, h]
                (startX, startY, endX, endY) = box.astype('int')
                cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 255, 0), 2)
                text = f'{label}: {confidence*100:.1f}%'
                text_y = max(startY - 10, 20)
                cv2.putText(frame, text, (startX, text_y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (255, 0, 0), 2)
        return frame

    def callback_usb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame_proc = self.procesar_imagen(frame)
        out_msg = self.bridge.cv2_to_imgmsg(frame_proc, encoding='bgr8')
        self.pub_usb.publish(out_msg)

    def callback_front(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame_proc = self.procesar_imagen(frame)
        out_msg = self.bridge.cv2_to_imgmsg(frame_proc, encoding='bgr8')
        self.pub_front.publish(out_msg)

    def callback_rear(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame_proc = self.procesar_imagen(frame)
        out_msg = self.bridge.cv2_to_imgmsg(frame_proc, encoding='bgr8')
        self.pub_rear.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DetectorObjetos()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
