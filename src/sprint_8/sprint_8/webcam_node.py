#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher_node')

        # Mejora 2: reducir cola
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw_genital', 1)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()

        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la webcam!")
        else:
            self.get_logger().info("Webcam inicializada correctamente.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("No se pudo leer el frame de la webcam")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

    # Mejora 5: liberación segura
    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap is not None:
            if self.cap.isOpened():
                self.cap.release()
                self.get_logger().info("Cámara liberada correctamente")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()