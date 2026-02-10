import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class OverheadWarehouseDetector(Node):
    def __init__(self):
        super().__init__('nodo_pattern_matching')

        # Suscripción a la cámara cenital
        self.subscription = self.create_subscription(
            Image,
            '/overhead_camera/image_raw',
            self.image_callback,
            10
        )

        # Publicador JSON tipo ArUco
        self.detections_pub = self.create_publisher(
            String,
            '/overhead_camera/warehouse_detections',
            10
        )

        # Imagen anotada
        self.image_pub = self.create_publisher(
            Image,
            '/overhead_camera/warehouse_annotated',
            10
        )

        self.bridge = CvBridge()

        # Cargar plantilla del almacén
        script_dir = os.path.dirname(os.path.realpath(__file__))
        template_path = os.path.join(script_dir, 'plantilla_almacen.png')
        self.template = cv2.imread(template_path, cv2.IMREAD_GRAYSCALE)

        if self.template is None:
            self.get_logger().error(f'No se pudo cargar la plantilla: {template_path}')
            self.template = np.zeros((50, 50), dtype=np.uint8)

        self.w, self.h = self.template.shape[::-1]
        self.threshold = 0.55   # Threshold más bajo para detectar mejor en Gazebo
        self.max_detections = 3

        self.get_logger().info('Detector de almacenes iniciado (hasta 3 detecciones)')

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo imagen: {e}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # matchTemplate
        res = cv2.matchTemplate(gray, self.template, cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

        self.get_logger().info(f"Max matchTemplate: {max_val:.2f} en {max_loc}")

        # Ubicaciones que superan el threshold
        locations = np.where(res >= self.threshold)
        scores = res[locations]

        detections = list(zip(locations[1], locations[0], scores))
        detections.sort(key=lambda x: x[2], reverse=True)

        selected = []

        for x, y, score in detections:
            if len(selected) >= self.max_detections:
                break

            overlap = False
            for sx, sy, _ in selected:
                if abs(x - sx) < self.w and abs(y - sy) < self.h:
                    overlap = True
                    break

            if not overlap:
                selected.append((x, y, score))

        # --- FALLBACK: si no hay selected, añadir la mejor coincidencia ---
        if len(selected) == 0 and max_val > 0.0:
            x, y = max_loc
            selected.append((x, y, max_val))

        # Construir JSON tipo ArUco
        out_detections = []
        for i, (x, y, score) in enumerate(selected):
            cx = x + self.w // 2
            cy = y + self.h // 2
            out_detections.append({
                "id": i,
                "px": float(cx),
                "py": float(cy)
            })

            # Dibujar en la imagen
            cv2.rectangle(cv_image, (x, y), (x + self.w, y + self.h), (0, 255, 0), 3)
            cv2.putText(cv_image, f"ID {i}", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        def fmt(d):
            return '{' + f'"id":{d["id"]},"px":{d["px"]:.2f},"py":{d["py"]:.2f}' + '}'

        msg_out = String()
        msg_out.data = '[' + ','.join(fmt(d) for d in out_detections) + ']'
        self.detections_pub.publish(msg_out)

        # Publicar imagen anotada
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = OverheadWarehouseDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
