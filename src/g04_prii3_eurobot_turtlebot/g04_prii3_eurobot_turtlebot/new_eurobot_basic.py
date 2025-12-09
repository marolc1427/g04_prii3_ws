import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import json


class OverheadArucoDetector(Node):
    def __init__(self):
        super().__init__('overhead_aruco_detector')

        # Suscripción a la cámara cenital (Gazebo plugin publica en /overhead_camera/image_raw)
        self.subscription = self.create_subscription(
            Image,
            '/overhead_camera/image_raw',
            self.image_callback,
            10
        )

        # Publicador de imagen anotada para RViz2
        self.image_annotated_pub = self.create_publisher(Image, '/overhead_camera/image_annotated', 10)

        # Publicadores por ID específico solicitados
        self.target_ids = [20, 21, 22, 23, 3, 8]
        self.id_publishers = {}
        for tid in self.target_ids:
            topic = f'/overhead_camera/aruco_{tid}'
            self.id_publishers[tid] = self.create_publisher(String, topic, 10)

        # Bridge ROS <-> OpenCV
        self.bridge = CvBridge()

        # Diccionario 4x4_50 (OpenCV 4.6)
        # Preferir getPredefinedDictionary en OpenCV >= 4.5
        try:
            self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        except AttributeError:
            self.dictionary = aruco.Dictionary_get(aruco.DICT_4X4_50)

        # Parámetros de detector, ajustados para robustez
        self.parameters = aruco.DetectorParameters_create()
        # Ampliar ventanas y tamaños para captar más marcadores
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 53
        self.parameters.adaptiveThreshWinSizeStep = 10
        self.parameters.minMarkerPerimeterRate = 0.02
        self.parameters.maxMarkerPerimeterRate = 6.0
        self.parameters.polygonalApproxAccuracyRate = 0.05
        self.parameters.minCornerDistanceRate = 0.03
        self.parameters.minDistanceToBorder = 1
        self.parameters.minMarkerDistanceRate = 0.03
        self.parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.parameters.cornerRefinementWinSize = 5
        self.parameters.cornerRefinementMaxIterations = 30
        self.parameters.cornerRefinementMinAccuracy = 0.1
       
        # Cámara cenital: matriz intrínseca aproximada y sin distorsión
        self.camera_matrix = np.array([
            [600.0, 0, 640.0],  # cx ~ mitad de 1280
            [0, 600.0, 360.0],  # cy ~ mitad de 720
            [0, 0, 1.0]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)

        # Tamaño del marcador en metros (ajústalo si es distinto)
        self.marker_length = 0.15

        self.get_logger().info('Suscrito a /overhead_camera/image_raw')
        self.get_logger().info('Publicadores por ID: ' + ', '.join([f'/overhead_camera/aruco_{tid}' for tid in self.target_ids]))

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo imagen: {e}')
            return

        # Preprocesamiento: usar escala de grises para mejorar contraste
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # Detectar marcadores ArUco con padding para captar los que están pegados al borde
        pad = 20  # píxeles de margen artificial
        gray_padded = cv2.copyMakeBorder(gray, pad, pad, pad, pad, borderType=cv2.BORDER_CONSTANT, value=255)
        # Primera pasada sobre imagen acolchada
        corners_p1, ids_p1, _ = aruco.detectMarkers(gray_padded, self.dictionary, parameters=self.parameters)

        # Segunda pasada con mejora de contraste (CLAHE)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray_clahe = clahe.apply(gray)
        gray_clahe_padded = cv2.copyMakeBorder(gray_clahe, pad, pad, pad, pad, borderType=cv2.BORDER_CONSTANT, value=255)
        corners_p2, ids_p2, _ = aruco.detectMarkers(gray_clahe_padded, self.dictionary, parameters=self.parameters)

        # Tercera pasada con ecualización global
        gray_eq = cv2.equalizeHist(gray)
        gray_eq_padded = cv2.copyMakeBorder(gray_eq, pad, pad, pad, pad, borderType=cv2.BORDER_CONSTANT, value=255)
        corners_p3, ids_p3, _ = aruco.detectMarkers(gray_eq_padded, self.dictionary, parameters=self.parameters)

        # Unir resultados evitando duplicados por ID
        merged_ids = []
        merged_corners = []
        def add_results(c, i):
            if i is None or len(i) == 0:
                return
            for idx, cid in enumerate(i):
                mid = int(cid[0])
                if mid not in merged_ids:
                    merged_ids.append(mid)
                    # remover padding de las esquinas detectadas
                    pts = c[idx]
                    pts_adjusted = pts.copy()
                    pts_adjusted[0][:, 0] -= pad
                    pts_adjusted[0][:, 1] -= pad
                    merged_corners.append(pts_adjusted)

        add_results(corners_p1, ids_p1)
        add_results(corners_p2, ids_p2)
        add_results(corners_p3, ids_p3)

        if len(merged_ids) > 0:
            # Reconstruir formato ids esperado por aruco APIs
            ids = np.array([[mid] for mid in merged_ids], dtype=np.int32)
            corners = merged_corners
        else:
            ids = None
            corners = []

        if ids is None or len(ids) == 0:
            # publicar imagen sin anotaciones para RViz
            try:
                self.image_annotated_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))
            except Exception:
                pass
            return

        # Estimar pose (opcional)
        try:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
        except Exception:
            rvecs, tvecs = None, None

        detections = []
        h, w = cv_image.shape[:2]

        def marker_valid(pts):
            # pts: (1,4,2)
            p = pts[0]
            # bounds check
            if np.any(p[:, 0] < 0) or np.any(p[:, 0] >= w) or np.any(p[:, 1] < 0) or np.any(p[:, 1] >= h):
                return False
            # side lengths
            l = []
            for j in range(4):
                a = p[j]
                b = p[(j + 1) % 4]
                l.append(np.linalg.norm(a - b))
            l = np.array(l)
            mean_l = np.mean(l)
            if mean_l < 15 or mean_l > 400:  # reject too small/large
                return False
            # squareness: low variance across side lengths
            if np.std(l) / (mean_l + 1e-6) > 0.35:
                return False
            # area within plausible range
            area = cv2.contourArea(p.astype(np.float32))
            if area < 300 or area > (w * h * 0.15):
                return False
            return True

        import math

        for i in range(len(ids)):
            # Validación de marcador para evitar falsos positivos extremos
            try:
                if not marker_valid(corners[i]):
                    continue
            except Exception:
                pass
            marker_id = int(ids[i][0])
            center = np.mean(corners[i][0], axis=0)
            cx, cy = float(center[0]), float(center[1])

            # Calcular orientación (yaw) a partir de las dos primeras esquinas.
            # Usamos la arista (corner 0 -> corner 1) para definir el eje local x del marcador
            # y calculamos el ángulo respecto al eje X de la imagen.
            try:
                p = corners[i][0]
                dx = float(p[1][0] - p[0][0])
                dy = float(p[1][1] - p[0][1])
                angle_rad = math.atan2(dy, dx)
                angle_deg = math.degrees(angle_rad)
                # Normalizar a [-180, 180]
                if angle_deg > 180.0:
                    angle_deg -= 360.0
                if angle_deg <= -180.0:
                    angle_deg += 360.0
            except Exception:
                angle_deg = 0.0

            # Publicar solo los campos solicitados: id, px, py, orientation (grados)
            det = {'id': marker_id, 'px': cx, 'py': cy, 'orientation': angle_deg}
            detections.append(det)

            # Si el ID está entre los solicitados, publicar en su topic dedicado
            if marker_id in self.target_ids:
                # Incluir sistema de ejes si hay pose estimada (rvec/tvec)
                if rvecs is not None and tvecs is not None:
                    try:
                        rv = rvecs[i][0].astype(float).tolist()
                        tv = tvecs[i][0].astype(float).tolist()
                    except Exception:
                        rv, tv = None, None
                else:
                    rv, tv = None, None

                # Redondear a 4 decimales para reducir tamaño y ruido
                rv_rounded = [round(float(x), 4) for x in rv] if rv is not None else None
                tv_rounded = [round(float(x), 4) for x in tv] if tv is not None else None

                msg_dict = {
                    'id': marker_id,
                    'px': round(cx, 4),
                    'py': round(cy, 4),
                    'orientation': round(angle_deg, 4),
                    'rvec': rv_rounded,
                    'tvec': tv_rounded
                }
                m = String()
                m.data = json.dumps(msg_dict, separators=(',', ':'))
                try:
                    self.id_publishers[marker_id].publish(m)
                except Exception:
                    pass

        # No se publica el agregado de detecciones en este nodo (solo por ID)

        # Dibujar marcadores y ejes en la imagen para RViz
        try:
            if ids is not None and len(ids) > 0:
                aruco.drawDetectedMarkers(cv_image, corners, ids)
            if rvecs is not None and tvecs is not None:
                for i in range(len(ids)):
                    cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], self.marker_length)
            # Publicar imagen anotada
            self.image_annotated_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = OverheadArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
