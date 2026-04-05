import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import math

class ArUcoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # Configuración de filtrado
        self.ignored_ids = {20, 21, 22, 23}
        # Definimos los 3 IDs que realmente nos interesan para los 3 tópicos
        self.target_ids = {1, 2, 3}  # Ajusta estos IDs a tus necesidades reales

        self.subscription = self.create_subscription(
            Image, '/image_warped', self.image_callback, 10)

        self.image_pub = self.create_publisher(Image, '/image_aruco_debug', 10)

        # Diccionario de publicadores para los 3 tópicos específicos
        self.string_publishers = {
            id_: self.create_publisher(String, f'/detection/aruco_{id_}', 10)
            for id_ in self.target_ids
        }

        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

    def calculate_angle(self, corners):
        """Calcula el ángulo de rotación del marcador en grados."""
        # Esquinas: [top-left, top-right, bottom-right, bottom-left]
        tl = corners[0]
        tr = corners[1]
        
        # Diferencia en Y y X entre la esquina superior derecha e izquierda
        dx = tr[0] - tl[0]
        dy = tr[1] - tl[1]
        
        # atan2 devuelve el ángulo en radianes. Convertimos a grados.
        angle_rad = math.atan2(dy, dx)
        return math.degrees(angle_rad)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids, _ = self.detector.detectMarkers(cv_image)

        if ids is not None:
            ids_list = ids.flatten().tolist()
            debug_image = cv_image.copy()
            cv2.aruco.drawDetectedMarkers(debug_image, corners, ids)

            for i, aruco_id in enumerate(ids_list):
                # 1. Filtro de exclusión
                if aruco_id in self.ignored_ids:
                    continue
                
                # 2. Solo publicamos si está en nuestra lista de interés
                if aruco_id in self.string_publishers:
                    # Cálculo de posición (centro)
                    center = np.mean(corners[i][0], axis=0)
                    
                    # Cálculo de ángulo
                    angle = self.calculate_angle(corners[i][0])

                    # Creación del objeto JSON
                    data = {
                        "id": int(aruco_id),
                        "x": round(float(center[0]), 2),
                        "y": round(float(center[1]), 2),
                        "angle": round(angle, 2)
                    }

                    msg_str = String()
                    msg_str.data = json.dumps(data)
                    
                    self.string_publishers[aruco_id].publish(msg_str)

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, "bgr8"))
        else:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = ArUcoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()