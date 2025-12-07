import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/overhead_camera/image_raw',
            self.image_callback,
            10  # cola de 10 imágenes
        )

        # Diccionario 4x4_50
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()

    def image_callback(self, msg):
        # Convertir ROS Image a OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.detect_aruco(frame)

    def detect_aruco(self, frame):
        # --- Preprocesamiento ---
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)  # reduce ruido
        gray = cv2.equalizeHist(gray)  # mejora contraste

        # --- Detección ---
        corners, ids, rejected = aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.parameters
        )

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            print("Detected IDs:", ids.flatten())

        cv2.imshow("Aruco Detection", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
