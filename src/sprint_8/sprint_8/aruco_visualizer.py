from __future__ import annotations
import json
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

class DetectorPiezasNode(Node):
    def __init__(self) -> None:
        super().__init__('detector_piezas_node')

        # Parámetros
        self.declare_parameter('image_topic', 'camera/color/image_raw')
        self.declare_parameter('out_topic', '/detection/pieza_aruco')
        self.declare_parameter('viz_topic', '/detection/image_viz')
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('ignored_ids', [20, 21, 22, 23])

        self.image_topic = self.get_parameter('image_topic').value
        self.out_topic = self.get_parameter('out_topic').value
        self.viz_topic = self.get_parameter('viz_topic').value
        self.ignored_ids = set(int(x) for x in self.get_parameter('ignored_ids').value)

        # ArUco Setup
        dictionary_name = str(self.get_parameter('dictionary').value)
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
        self._init_aruco_backend()

        self.bridge = CvBridge()
        
        # Publishers
        self.publisher = self.create_publisher(String, self.out_topic, 10)
        self.viz_publisher = self.create_publisher(Image, self.viz_topic, 10)
        
        # Subscriber
        self.subscription = self.create_subscription(Image, self.image_topic, self.image_callback, 10)

        self.get_logger().info(f"Visualización disponible en: {self.viz_topic}")

    def _init_aruco_backend(self) -> None:
        """Maneja compatibilidad de versiones de OpenCV."""
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, cv2.aruco.DetectorParameters())
            self._detect_markers = lambda img: self.detector.detectMarkers(img)
        else:
            self.params = cv2.aruco.DetectorParameters_create()
            self._detect_markers = lambda img: cv2.aruco.detectMarkers(img, self.dictionary, parameters=self.params)

    def image_callback(self, msg: Image) -> None:
        try:
            # Convertimos a BGR para procesar y dibujar
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error conversión: {e}")
            return

        corners, ids, _ = self._detect_markers(cv_image)

        if ids is not None:
            h, w = cv_image.shape[:2]
            img_center = (w // 2, h // 2)

            # Dibujar centro de la imagen para referencia de error
            cv2.drawMarker(cv_image, img_center, (0, 0, 255), cv2.MARKER_CROSS, 20, 2)

            for i, aruco_id in enumerate(ids.flatten()):
                if int(aruco_id) in self.ignored_ids:
                    continue

                # Lógica de detección y error
                marker_center = np.mean(corners[i][0], axis=0).astype(int)
                error_x = float(marker_center[0] - img_center[0])
                error_y = float(marker_center[1] - img_center[1])

                # Publicar JSON
                data = {'id': int(aruco_id), 'error_x': round(error_x, 2), 'error_y': round(error_y, 2)}
                self.publisher.publish(String(data=json.dumps(data)))

                # --- VISUALIZACIÓN ---
                # Dibujar caja del ArUco
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                # Dibujar línea de error desde el centro
                cv2.line(cv_image, img_center, tuple(marker_center), (0, 255, 0), 2)
                cv2.putText(cv_image, f"Err: {int(error_x)},{int(error_y)}", 
                            (marker_center[0], marker_center[1]-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # Publicar la imagen anotada para RViz
        viz_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        viz_msg.header = msg.header # Mantener timestamp y frame_id
        self.viz_publisher.publish(viz_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DetectorPiezasNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()