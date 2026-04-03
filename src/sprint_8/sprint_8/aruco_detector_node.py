import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArUcoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # Suscripción a la homografía
        self.subscription = self.create_subscription(
            Image, '/image_warped', self.image_callback, 10)

        # Publicador de la imagen con dibujos (Visualizador)
        self.image_pub = self.create_publisher(Image, '/image_aruco_debug', 10)

        # Diccionario de publicadores dinámicos para los Strings
        self.string_publishers = {}

        self.bridge = CvBridge()
        
        # Configuración ArUco (Humble usa ArucoDetector)
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

    def get_aruco_publisher(self, aruco_id):
        """Crea un publicador si no existe para ese ID específico."""
        topic_name = f'/detection/aruco{aruco_id}'
        if aruco_id not in self.string_publishers:
            self.get_logger().info(f"Creando nuevo tópico: {topic_name}")
            self.string_publishers[aruco_id] = self.create_publisher(String, topic_name, 10)
        return self.string_publishers[aruco_id]

    def image_callback(self, msg):
        # Convertir imagen de ROS a OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Detección
        corners, ids, _ = self.detector.detectMarkers(cv_image)

        if ids is not None:
            ids_list = ids.flatten().tolist()
            
            # Dibujar marcadores sobre una copia para el visualizador
            debug_image = cv_image.copy()
            cv2.aruco.drawDetectedMarkers(debug_image, corners, ids)

            for i, aruco_id in enumerate(ids_list):
                # Calcular centro en la imagen de homografía
                center = np.mean(corners[i][0], axis=0)
                
                # Publicar el String en el tópico correspondiente
                msg_str = String()
                msg_str.data = f"ID: {aruco_id} | Posicion Tablero: X={int(center[0])}, Y={int(center[1])}"
                
                pub = self.get_aruco_publisher(aruco_id)
                pub.publish(msg_str)

            # Publicar la imagen de debug
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, "bgr8"))
        else:
            # Si no hay IDs, publicamos la imagen limpia
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = ArUcoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()