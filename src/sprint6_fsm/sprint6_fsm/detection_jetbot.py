import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import json

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Suscriptor para las imágenes de la cámara
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publicador para visualización (opcional)
        self.image_pub = self.create_publisher(Image, '/aruco_detection_image', 10)

        # Publicador: detecciones (JSON) en imagen reescalada
        self.aruco_pub = self.create_publisher(String, '/aruco_detections', 10)

        # Bridge para convertir entre ROS Image y OpenCV
        self.bridge = CvBridge()

        # Resolución estándar (OpenCV resize usa (width, height))
        # El requisito se expresa como 720x1080 (alto x ancho)
        self.target_height = 720
        self.target_width = 1080

        # Procesar a 2 Hz para no saturar la Jetson
        self.process_rate_hz = 2.0
        self.latest_image_msg = None
        self.last_processed_stamp = None
        self.process_timer = self.create_timer(1.0 / self.process_rate_hz, self.process_latest_image)

        # Parámetros de detección ArUco - DICCIONARIO 5X5
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
        self.parameters = aruco.DetectorParameters_create()

        # Hacer el detector más sensible para 5x5
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 23
        self.parameters.adaptiveThreshWinSizeStep = 10
        self.parameters.minMarkerPerimeterRate = 0.03
        self.parameters.maxMarkerPerimeterRate = 4.0
        self.parameters.polygonalApproxAccuracyRate = 0.05
        self.parameters.minCornerDistanceRate = 0.05
        self.parameters.minDistanceToBorder = 3
        self.parameters.minMarkerDistanceRate = 0.05
        self.parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.parameters.cornerRefinementWinSize = 5
        self.parameters.cornerRefinementMaxIterations = 30
        self.parameters.cornerRefinementMinAccuracy = 0.1

        # Parámetros de la cámara del JetBot (valores aproximados)
        self.camera_matrix = np.array([
            [600.0, 0, 320.0],
            [0, 600.0, 240.0],
            [0, 0, 1.0]
        ], dtype=np.float32)

        self.dist_coeffs = np.zeros((4, 1))

        # Tamaño del marcador en metros
        self.marker_length = 0.05

        # Variable para controlar la impresión en terminal
        self.last_detected_ids = []

        # Contador de frames para diagnóstico
        self.frame_count = 0

        self.get_logger().info("Nodo de detección de ArUcos inicializado")
        self.get_logger().info("Suscrito a /camera/image_raw")
        self.get_logger().info("Publicando detecciones en /aruco_detections (std_msgs/String JSON)")
        self.get_logger().info(f"Reescalando a {self.target_height}x{self.target_width} (alto x ancho)")
        self.get_logger().info(f"Procesando a {self.process_rate_hz} Hz")

    def image_callback(self, msg):
        # Guardamos la última imagen recibida y la procesamos con un timer a 2 Hz.
        # Así evitamos convertir/re-escalar/detectar a la tasa completa de la cámara.
        self.frame_count += 1
        self.latest_image_msg = msg

    def process_latest_image(self):
        msg = self.latest_image_msg
        if msg is None:
            return

        stamp = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        if stamp == self.last_processed_stamp:
            return
        self.last_processed_stamp = stamp

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo imagen: {e}')
            return

        resized = cv2.resize(
            cv_image,
            (self.target_width, self.target_height),
            interpolation=cv2.INTER_AREA
        )

        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

        corners, ids, _rejected = aruco.detectMarkers(
            gray, self.dictionary, parameters=self.parameters
        )

        output_image = resized.copy()
        detections = []
        current_detected_ids = []

        if ids is not None and len(ids) > 0:
            current_detected_ids = ids.flatten().tolist()

            if current_detected_ids != self.last_detected_ids:
                self.get_logger().info(f"ArUco IDs detectados: {current_detected_ids}")
                self.last_detected_ids = current_detected_ids

            aruco.drawDetectedMarkers(output_image, corners, ids)

            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                pts = corners[i][0]  # (4,2)
                center = np.mean(pts, axis=0)
                px, py = float(center[0]), float(center[1])

                cv2.putText(
                    output_image,
                    f"ID:{marker_id} ({int(px)},{int(py)})",
                    (max(0, int(px) - 80), max(0, int(py) - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

                detections.append({
                    'id': marker_id,
                    'px': px,
                    'py': py,
                    'corners': pts.astype(float).tolist(),
                })
        else:
            if self.last_detected_ids:
                self.last_detected_ids = []
                self.get_logger().info("No se detectan ArUcos")

            if self.frame_count % 50 == 0:
                self.get_logger().info(f"Frame {self.frame_count}: esperando detección de ArUco...")

        payload = {
            'width': int(self.target_width),
            'height': int(self.target_height),
            'stamp': {
                'sec': int(msg.header.stamp.sec),
                'nanosec': int(msg.header.stamp.nanosec),
            },
            'frame_id': msg.header.frame_id,
            'detections': detections,
        }
        self.aruco_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))

        try:
            detection_msg = self.bridge.cv2_to_imgmsg(output_image, encoding='bgr8')
            detection_msg.header = msg.header
            self.image_pub.publish(detection_msg)
        except Exception as e:
            self.get_logger().error(f'Error publicando imagen: {e}')

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()

    try:
        rclpy.spin(aruco_detector)
    except KeyboardInterrupt:
        pass
    finally:
        aruco_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()