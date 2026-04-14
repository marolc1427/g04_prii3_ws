#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np
import json
import math

class ArUcoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # Configuración de filtrado
        self.ignored_ids = {}
        self.target_ids = {3,20,21,22,23} 

        # Suscripción a la imagen sin la homografía
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw_genital', self.image_callback, 10)

        self.image_pub = self.create_publisher(Image, '/image_aruco_debug', 10)

        # Diccionario de publicadores específicos
        self.string_publishers = {
            id_: self.create_publisher(String, f'/detection/aruco_{id_}', 10)
            for id_ in self.target_ids
        }

        self.bridge = CvBridge()

        # --- CAMBIO PARA FOXY / OPENCV 4.6 ---
        # Usamos el API antiguo de ArUco
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()

        self.get_logger().info("✅ Detector de ArUcos post-homografía iniciado (Foxy)")

    def calculate_angle(self, corners):
        """
        Calcula el ángulo de rotación del marcador.
        corners viene como [4, 2] -> [top-left, top-right, bottom-right, bottom-left]
        """
        tl = corners[0]
        tr = corners[1]
        
        dx = tr[0] - tl[0]
        dy = tr[1] - tl[1]
        
        # atan2 para obtener el ángulo en el plano de la imagen
        angle_rad = math.atan2(dy, dx)
        return math.degrees(angle_rad)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # --- DETECCIÓN API FOXY ---
        corners, ids, _ = cv2.aruco.detectMarkers(
            cv_image, self.dictionary, parameters=self.parameters)

        if ids is not None:
            ids_list = ids.flatten().tolist()
            debug_image = cv_image.copy()
            cv2.aruco.drawDetectedMarkers(debug_image, corners, ids)

            for i, aruco_id in enumerate(ids_list):
                # 1. Filtro: Ignoramos los que forman el tablero
                if aruco_id in self.ignored_ids:
                    continue
                
                # 2. Publicamos si es uno de nuestros objetivos (1, 2 o 3)
                if aruco_id in self.target_ids:
                    # Cálculo de posición (promedio de las 4 esquinas)
                    center = np.mean(corners[i][0], axis=0)
                    
                    # Cálculo de ángulo relativo al eje X de la imagen
                    angle = self.calculate_angle(corners[i][0])

                    # Empaquetado JSON
                    data = {
                        "id": int(aruco_id),
                        "x": round(float(center[0]), 2),
                        "y": round(float(center[1]), 2),
                        "angle": round(angle, 2)
                    }

                    msg_str = String()
                    msg_str.data = json.dumps(data)
                    
                    self.string_publishers[aruco_id].publish(msg_str)
                    # self.get_logger().info(f"Publicando ID {aruco_id}")

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, "bgr8"))
        else:
            # Si no hay marcadores, publicamos la imagen limpia para el debug
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = ArUcoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()