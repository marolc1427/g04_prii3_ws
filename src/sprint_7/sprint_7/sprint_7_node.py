#!/usr/bin/env python3
import time
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image


@dataclass
class LineDetection:
    found: bool = False
    cx: float = 0.0
    cy: float = 0.0
    error_px: float = 0.0
    error_norm: float = 0.0
    area: float = 0.0


class LineFollowerPID(Node):
    def __init__(self):
        super().__init__("line_follower_pid")

        self.image_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.bridge = CvBridge()

        self.target_width = 640
        self.target_height = 480

        self.roi_y_start_ratio = 0.70
        self.roi_y_end_ratio = 0.95

        self.process_rate_hz = 10.0
        self.latest_image_msg = None
        self.last_processed_stamp = None
        self.process_timer = self.create_timer(1.0 / self.process_rate_hz, self.process_latest_image)

        self.binary_threshold = 85

        self.min_contour_area = 250.0
        self.max_center_jump_px = 180.0

        self.kp = 1.25
        self.kd = 0.35
        self.ki = 0.00

        self.base_speed = 0.10
        self.min_speed = 0.035
        self.max_speed = 0.12

        self.max_angular = 1.4
        self.speed_reduction_gain = 0.07

        self.line_lost_timeout_s = 0.35
        self.search_angular = 0.55
        self.search_speed = 0.02

        self.last_detection = LineDetection()
        self.prev_error = 0.0
        self.error_integral = 0.0
        self.prev_time = time.time()

        self.last_line_seen_time = 0.0
        self.last_valid_cx = None
        self.last_turn_sign = 1.0

        self.frame_count = 0

        self.get_logger().info("Nodo line_follower_pid iniciado")
        self.get_logger().info("Suscrito a /camera/image_raw")
        self.get_logger().info("Publicando velocidad en /cmd_vel")

    def image_callback(self, msg):
        self.latest_image_msg = msg
        self.frame_count += 1

    def process_latest_image(self):
        msg = self.latest_image_msg
        if msg is None:
            self.publish_stop()
            return

        stamp = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        if stamp == self.last_processed_stamp:
            return
        self.last_processed_stamp = stamp

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Error convirtiendo imagen: {e}")
            self.publish_stop()
            return

        resized = cv2.resize(
            cv_image,
            (self.target_width, self.target_height),
            interpolation=cv2.INTER_AREA
        )

        detection = self.detect_line(resized)
        self.last_detection = detection

        now = time.time()
        dt = now - self.prev_time
        if dt <= 1e-6:
            dt = 1e-3

        if detection.found:
            self.last_line_seen_time = now
            self.last_valid_cx = detection.cx

            error = detection.error_norm
            derivative = (error - self.prev_error) / dt
            self.error_integral += error * dt

            control = (
                self.kp * error +
                self.kd * derivative +
                self.ki * self.error_integral
            )

            angular_z = -control
            angular_z = self.clamp(angular_z, -self.max_angular, self.max_angular)

            speed = self.base_speed - self.speed_reduction_gain * abs(error)
            speed = self.clamp(speed, self.min_speed, self.max_speed)

            cmd = Twist()
            cmd.linear.x = float(speed)
            cmd.angular.z = float(angular_z)
            self.cmd_pub.publish(cmd)

            if error > 0:
                self.last_turn_sign = 1.0
            elif error < 0:
                self.last_turn_sign = -1.0

            self.prev_error = error
            self.prev_time = now

        else:
            time_since_seen = now - self.last_line_seen_time

            if time_since_seen <= self.line_lost_timeout_s:
                cmd = Twist()
                cmd.linear.x = float(self.search_speed)
                cmd.angular.z = float(-self.last_turn_sign * self.search_angular)
                self.cmd_pub.publish(cmd)
            else:
                self.publish_stop()

            self.prev_error = 0.0
            self.error_integral = 0.0
            self.prev_time = now

    def detect_line(self, image_bgr):
        h, w = image_bgr.shape[:2]

        y1 = int(h * self.roi_y_start_ratio)
        y2 = int(h * self.roi_y_end_ratio)
        roi = image_bgr[y1:y2, :]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        _, mask = cv2.threshold(
            blur,
            self.binary_threshold,
            255,
            cv2.THRESH_BINARY_INV
        )

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        valid_candidates = []
        roi_center_x = w / 2.0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_contour_area:
                continue

            m = cv2.moments(contour)
            if m["m00"] == 0:
                continue

            cx = float(m["m10"] / m["m00"])
            cy = float(m["m01"] / m["m00"])

            valid_candidates.append({
                "contour": contour,
                "area": area,
                "cx": cx,
                "cy": cy
            })

        detection = LineDetection(found=False)

        if not valid_candidates:
            return detection

        selected = self.select_best_contour(valid_candidates, roi_center_x)

        cx = selected["cx"]
        cy = selected["cy"]
        area = selected["area"]

        global_cy = cy + y1
        error_px = cx - roi_center_x
        error_norm = error_px / roi_center_x

        detection = LineDetection(
            found=True,
            cx=cx,
            cy=global_cy,
            error_px=error_px,
            error_norm=error_norm,
            area=area
        )

        return detection

    def select_best_contour(self, candidates, roi_center_x):
        if self.last_valid_cx is None:
            return min(candidates, key=lambda c: abs(c["cx"] - roi_center_x))

        close_candidates = [
            c for c in candidates
            if abs(c["cx"] - self.last_valid_cx) <= self.max_center_jump_px
        ]

        if close_candidates:
            return min(close_candidates, key=lambda c: abs(c["cx"] - self.last_valid_cx))

        return min(candidates, key=lambda c: abs(c["cx"] - roi_center_x))

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def clamp(value, vmin, vmax):
        return max(vmin, min(value, vmax))


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerPID()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()