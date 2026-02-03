import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
import time
import math

class AutonomousArucoRoute(Node):
    """
    Nodo autónomo que recorre los ArUcos 20->21->22->23 usando el robot marcado por ArUco 3.

    - ArUcos estáticos (20,21,22,23): se leen UNA sola vez al inicio y se guardan.
    - ArUco del robot (3): se lee continuamente para obtener pose y orientación.
    - Control de movimiento continuo (proporcional) publicando en /cmd_vel.
    - Publicación de velocidad limitada a 1 Hz (timer ROS2).
    """

    def __init__(self):
        super().__init__('new_aruco_go_to')

        self.robot_id = 3
        self.static_ids = [20, 21, 22, 23]

        # Publicador de velocidad del robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Pose del robot (actualizada continuamente)
        self.robot = None  # {'px':float,'py':float,'orientation':float,'last_seen':float}

        # ArUcos estáticos (capturados una sola vez)
        self.static_markers = {}  # {id: {'px':float,'py':float}}
        self._static_subs = {}    # {id: Subscription} (se destruyen al capturar)

        # Suscripción continua SOLO del robot
        self.create_subscription(
            String,
            f'/overhead_camera/aruco_{self.robot_id}',
            self.robot_cb,
            10
        )

        # Suscripciones temporales a estáticos (hasta recibir 1ª medida)
        for tid in self.static_ids:
            topic = f'/overhead_camera/aruco_{tid}'
            sub = self.create_subscription(
                String,
                topic,
                lambda msg, tid=tid: self.static_aruco_cb(msg, tid),
                10,
            )
            self._static_subs[tid] = sub

        # Ruta a seguir (sin intervención del usuario)
        self.route = [20, 21, 22, 23]
        self.route_index = 0

        # Parámetros de control (proporcionales + saturación)
        self.max_linear_speed = 0.12  # m/s
        self.max_angular_speed = 0.5  # rad/s
        self.linear_kp = 0.0020       # (m/s)/px
        self.angular_kp = 0.012       # (rad/s)/deg
        self.distance_threshold_px = 30.0
        self.orientation_offset_deg = 270.0  # Compensa la diferencia entre orientación del ArUco y el heading real del robot.
        self.robot_timeout_s = 0.8
        self.spin_angular_speed = 0.8 * self.max_angular_speed
        self.spin_duration_s = math.pi / max(1e-6, abs(self.spin_angular_speed))
        self.spin_until = None

        # Publicar comando como mucho a 1 Hz
        self.control_timer = self.create_timer(1.0, self.control_step)

        self.get_logger().info(
            'AutonomousArucoRoute started: acquiring static ArUcos and following route '
            + '->'.join(map(str, self.route))
        )

    def _parse_aruco_json(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return None
        try:
            px = float(data.get('px', 0.0))
            py = float(data.get('py', 0.0))
            orientation = float(data.get('orientation', 0.0))
        except Exception:
            return None
        return px, py, orientation

    def static_aruco_cb(self, msg: String, tid: int):
        """Lee una sola vez el ArUco estático tid y destruye la suscripción."""
        if tid in self.static_markers:
            return

        parsed = self._parse_aruco_json(msg)
        if parsed is None:
            return

        px, py, _orientation = parsed
        self.static_markers[tid] = {'px': px, 'py': py}

        sub = self._static_subs.pop(tid, None)
        if sub is not None:
            self.destroy_subscription(sub)

        self.get_logger().info(f'Static ArUco {tid} captured at (px,py)=({px:.1f},{py:.1f})')

    def robot_cb(self, msg: String):
        """Callback continuo del ArUco del robot (ID 3)."""
        parsed = self._parse_aruco_json(msg)
        if parsed is None:
            return

        px, py, orientation = parsed
        self.robot = {'px': px, 'py': py, 'orientation': orientation, 'last_seen': time.time()}

        if not getattr(self, '_robot_detect_log', False):
            self.get_logger().info(f'Robot ArUco {self.robot_id} detected at (px,py)=({px:.1f},{py:.1f})')
            self._robot_detect_log = True

    def publish_twist(self, linear_x=0.0, angular_z=0.0):
        t = Twist()
        t.linear.x = float(linear_x)
        t.linear.y = 0.0
        t.linear.z = 0.0
        t.angular.x = 0.0
        t.angular.y = 0.0
        t.angular.z = float(angular_z)
        self.cmd_pub.publish(t)

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min(value, max_value), min_value)

    @staticmethod
    def _normalize_angle_deg(angle_deg: float) -> float:
        while angle_deg > 180.0:
            angle_deg -= 360.0
        while angle_deg < -180.0:
            angle_deg += 360.0
        return angle_deg

    def control_step(self):
        """Paso de control (1 Hz): calcula y publica Twist de forma simultánea."""

        # Esperar si aún no tenemos todos los ArUcos estáticos
        if len(self.static_markers) < len(self.static_ids):
            self.publish_twist(0.0, 0.0)
            if not hasattr(self, '_startup_log_counter'):
                self._startup_log_counter = 0
            self._startup_log_counter += 1
            if self._startup_log_counter % 5 == 0:
                missing = [tid for tid in self.static_ids if tid not in self.static_markers]
                self.get_logger().info(f'Waiting for static ArUcos: {missing}')
            return

        # Ruta finalizada
        if self.route_index >= len(self.route):
            if not getattr(self, '_done_logged', False):
                self.get_logger().info('Route complete. Stopping.')
                self._done_logged = True
            self.publish_twist(0.0, 0.0)
            return

        target_id = self.route[self.route_index]
        target = self.static_markers.get(target_id)
        if target is None:
            self.publish_twist(0.0, 0.0)
            self.get_logger().info(f'Waiting: target ArUco {target_id} not captured yet')
            return

        now = time.time()
        if self.robot is None or (now - self.robot.get('last_seen', 0.0) > self.robot_timeout_s):
            if not getattr(self, '_robot_timeout_logged', False):
                self.get_logger().warning('Robot pose unavailable or timeout; stopping.')
                self._robot_timeout_logged = True
            self.publish_twist(0.0, 0.0)
            return

        self._robot_timeout_logged = False

        # Coordenadas en imagen (px,py)
        robot_x_img = self.robot['px']
        robot_y_img = self.robot['py']
        target_x_img = target['px']
        target_y_img = target['py']

        # Convertir a coords matemáticas (y arriba)
        robot_x = robot_x_img
        robot_y = -robot_y_img
        target_x = target_x_img
        target_y = -target_y_img

        dx = target_x - robot_x
        dy = target_y - robot_y
        distance_px = math.hypot(dx, dy)

        # Llegada por distancia
        if distance_px < self.distance_threshold_px:
            self.publish_twist(0.0, 0.0)
            self.get_logger().info(f'Arrived at ArUco {target_id} (distance threshold). Initiating 180° spin.')
            self.spin_until = time.time() + self.spin_duration_s
            self.route_index += 1
            return

        target_angle_deg = math.degrees(math.atan2(dy, dx))
        raw_orientation_deg = float(self.robot.get('orientation', 0.0))
        robot_orientation_deg = -raw_orientation_deg + self.orientation_offset_deg
        robot_orientation_deg = self._normalize_angle_deg(robot_orientation_deg)
        angle_error_deg = self._normalize_angle_deg(target_angle_deg - robot_orientation_deg)

        # Control proporcional simultáneo
        angular_z = -self.angular_kp * angle_error_deg
        angular_z = self._clamp(angular_z, -self.max_angular_speed, self.max_angular_speed)

        linear_x = self.linear_kp * distance_px
        linear_scale = max(0.0, 1.0 - abs(angle_error_deg) / 180.0)
        linear_x *= linear_scale
        linear_x = self._clamp(linear_x, 0.0, self.max_linear_speed)

        self.get_logger().info(
            f'Target {target_id}: dist_px={distance_px:.1f}, target_angle={target_angle_deg:.1f}, '
            f'robot_heading={robot_orientation_deg:.1f}, angle_error={angle_error_deg:.1f}, '
            f'cmd_linear={linear_x:.3f}, cmd_angular={angular_z:.3f}'
        )

        self.publish_twist(linear_x, angular_z)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousArucoRoute()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_twist(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
