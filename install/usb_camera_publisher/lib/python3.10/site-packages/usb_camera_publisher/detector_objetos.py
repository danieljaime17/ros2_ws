import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class DetectorObjetos(Node):
    def __init__(self):
        super().__init__('detector_objetos')

        self.subscription = self.create_subscription(
            Image,
            '/usb_camera/image_raw',
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(Image, '/usb_camera/image_annotated', 10)
        self.bridge = CvBridge()

        # Rutas de los modelos
        model_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'models/')
        model_weights = os.path.join(model_path, 'frozen_inference_graph.pb')
        model_config = os.path.join(model_path, 'ssd_mobilenet_v1_coco.pbtxt')

        # Cargar modelo de detección pre-entrenado (MobileNet SSD, formato TensorFlow)
        self.net = cv2.dnn.readNetFromTensorflow(model_weights, model_config)

        # Ejemplo simple de clases COCO. Para más etiquetas usar el archivo completo de labels.
        self.labels = {
            1: 'person', 2: 'bicycle', 3: 'car', 4: 'motorcycle', 5: 'airplane', 6: 'bus',
            7: 'train', 8: 'truck', 9: 'boat', 17: 'cat', 18: 'dog', 77: 'cell phone'
        }

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
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
                # Ajusta para que el texto no se salga de la imagen
                text_y = max(startY - 10, 20)
                cv2.putText(frame, text, (startX, text_y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (255, 0, 0), 2)

        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DetectorObjetos()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

