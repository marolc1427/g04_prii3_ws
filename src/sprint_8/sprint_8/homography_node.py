#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class DebugHomography(Node):
    def __init__(self):
        super().__init__('debug_homography_node')
        
        # Suscripciones y Publicaciones
        self.subscription = self.create_subscription(Image, '/camera/image_raw_genital', self.image_callback, 10)
        self.publisher = self.create_publisher(Image, '/image_warped', 10)
        
        # NUEVO: Tópico para ver qué ArUcos detecta la cámara en tiempo real
        self.debug_pub = self.create_publisher(Image, '/image_debug', 10)
        
        self.bridge = CvBridge()
        self.last_log_time = 0

        # ArUco Foxy
        self.dic = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters_create()

        # Dimensiones
        self.esc = 5
        self.w_u, self.h_u = 180 * self.esc, 80 * self.esc
        self.m = 60 * self.esc
        self.w_tot, self.h_tot = int(self.w_u + 2 * self.m), int(self.h_u + 2 * self.m)

        self.get_logger().info("🛠️ MODO DEBUG INICIADO")

    def order_pts(self, pts):
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        rect[0], rect[2] = pts[np.argmin(s)], pts[np.argmax(s)]
        diff = np.diff(pts, axis=1)
        rect[1], rect[3] = pts[np.argmin(diff)], pts[np.argmax(diff)]
        return rect

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Aplicamos mejora de contraste para ayudar a la detección
        gray = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)).apply(gray)

        # Detección
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dic, parameters=self.params)

        # --- LÓGICA DE DEBUG ---
        now = time.time()
        should_log = (now - self.last_log_time > 2.0) # Loguear cada 2 segundos

        if ids is not None:
            ids_list = ids.flatten().tolist()
            
            # Dibujamos los marcadores detectados para verlos en RViz
            debug_frame = frame.copy()
            cv2.aruco.drawDetectedMarkers(debug_frame, corners, ids)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_frame, "bgr8"))

            ref_ids = [23, 22, 20, 21]
            missing = [rid for rid in ref_ids if rid not in ids_list]

            if not missing:
                if should_log: self.get_logger().info("✅ ¡Los 4 marcadores detectados! Publicando homografía...")
                
                centers_dict = {ids_list[i]: corners[i][0].mean(axis=0) for i in range(len(ids_list))}
                src_pts = np.array([centers_dict[rid] for rid in ref_ids], dtype="float32")
                src = self.order_pts(src_pts)
                
                dst = np.array([[self.m, self.m], [self.w_u + self.m, self.m], 
                                [self.w_u + self.m, self.h_u + self.m], [self.m, self.h_u + self.m]], dtype="float32")

                M, _ = cv2.findHomography(src, dst)
                warped = cv2.warpPerspective(frame, M, (self.w_tot, self.h_tot))
                self.publisher.publish(self.bridge.cv2_to_imgmsg(warped, "bgr8"))
            else:
                if should_log: self.get_logger().warn(f"⚠️ Detecto {ids_list}, pero faltan: {missing}")
        else:
            if should_log: self.get_logger().error("❌ No detecto NINGÚN ArUco. Revisa la iluminación o el enfoque.")
            # Publicamos la imagen original al debug para saber que el callback funciona
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

        if should_log: self.last_log_time = now

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DebugHomography())
    rclpy.shutdown()

if __name__ == '__main__':
    main()