import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
import threading
import time
import math


class AutonomousArucoRoute(Node):
    """Nodo autónomo que recorre los ArUcos 20->21->22->23 usando el robot marcado por ArUco 3.

    - Lee únicamente los topics publicados por `new_eurobot_basic` para los IDs
      3, 20, 21, 22, 23: `/overhead_camera/aruco_{id}` (std_msgs/String con JSON).
    - Controla el robot publicando en `/cmd_vel` (geometry_msgs/Twist).
    - Para navegar: gira hasta apuntar al objetivo y luego avanza en línea recta.
        - La llegada se determina UNICAMENTE por la distancia al marcador (umbral
            `distance_threshold_px`). Si el marcador no está visible momentáneamente
            el nodo espera hasta que vuelva a ser detectado.
    """

    def __init__(self):
        super().__init__('new_aruco_go_to')

        # IDs que queremos escuchar (según petición): robot=3 y objetivos 20,21,22,23
        self.listen_ids = [3, 20, 21, 22, 23]

        # Publicador de velocidad del robot (Waffle/Turtlebot plugin escucha /cmd_vel)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Suscripciones por ID
        for tid in self.listen_ids:
            topic = f'/overhead_camera/aruco_{tid}'
            self.create_subscription(String, topic, lambda msg, tid=tid: self.aruco_cb(msg, tid), 10)

        # Estructura de detecciones: {id: {'px':float,'py':float,'orientation':float,'last_seen':float}}
        self.raw = {}
        self.lock = threading.Lock()

        # Ruta a seguir (sin intervención del usuario)
        self.route = [20, 21, 22, 23]
        self.route_index = 0

        # Parámetros de control
        self.forward_speed = 0.12  # m/s
        self.turn_speed_rad = 0.5  # rad/s (límite)
        self.turn_kp = 0.012  # ganancia proporcional para giro
        self.angle_tolerance_deg = 6.0
        self.distance_threshold_px = 30.0

        # Nota: ya no usamos llegada por "silencio" de topic; solo distancia
        # (se mantiene la marca temporal por si se quiere debuggear detecciones)
        self.silence_timeout = 0.8

        # Estado
        self.running = True
        self.is_navigating = False

        # Hilo de control
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()

        self.get_logger().info('AutonomousArucoRoute started: following route ' + '->'.join(map(str, self.route)))

    def aruco_cb(self, msg: String, tid: int):
        """Callback por tópico `/overhead_camera/aruco_{id}`.
        Espera un JSON en `msg.data` con campos: id, px, py, orientation, ...
        """
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        try:
            px = float(data.get('px', 0.0))
            py = float(data.get('py', 0.0))
            orientation = float(data.get('orientation', 0.0))
        except Exception:
            return

        with self.lock:
            self.raw[tid] = {'px': px, 'py': py, 'orientation': orientation, 'last_seen': time.time()}

    def publish_twist(self, linear_x=0.0, angular_z=0.0):
        t = Twist()
        t.linear.x = float(linear_x)
        t.linear.y = 0.0
        t.linear.z = 0.0
        t.angular.x = 0.0
        t.angular.y = 0.0
        t.angular.z = float(angular_z)
        self.cmd_pub.publish(t)

    def calculate_navigation_vector(self, target_id):
        """Calcula (angle_error_deg, distance_px) entre robot (aruco 3) y target.

        Retorna:
        - ('robot_missing', None) si robot no visible
        - ('target_missing', None) si objetivo no visible (se espera)
        - (angle_error_deg, distance_px) en caso normal
        """
        now = time.time()
        with self.lock:
            robot = self.raw.get(3)
            target = self.raw.get(target_id)

        # Si target no visible → reportar y esperar (NO declarar llegada)
        if target is None:
            return 'target_missing', None

        # Si robot no visible
        if robot is None or (now - robot.get('last_seen', 0.0) > self.silence_timeout):
            return 'robot_missing', None

        # Coordenadas en imagen (px,py)
        robot_x_img = robot['px']
        robot_y_img = robot['py']
        target_x_img = target['px']
        target_y_img = target['py']

        # Convertir a coordenadas de mundo como hace el código original (invertir signos)
        robot_x = -robot_x_img
        robot_y = -robot_y_img
        target_x = -target_x_img
        target_y = -target_y_img

        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.hypot(dx, dy)

        target_angle_rad = math.atan2(dy, dx)
        target_angle_deg = math.degrees(target_angle_rad)

        robot_orientation = robot.get('orientation', 0.0)
        angle_error = target_angle_deg - robot_orientation

        # Normalizar a [-180, 180]
        while angle_error > 180:
            angle_error -= 360
        while angle_error < -180:
            angle_error += 360

        return angle_error, distance

    def _control_loop(self):
        rate = 10.0
        dt = 1.0 / rate

        # Comenzar la primera navegación automáticamente
        self.is_navigating = True

        while self.running:
            if not self.is_navigating:
                # No hay navegación activa: detener robot y dormir
                self.publish_twist(0.0, 0.0)
                time.sleep(dt)
                continue

            if self.route_index >= len(self.route):
                self.get_logger().info('Route complete. Stopping.')
                self.publish_twist(0.0, 0.0)
                self.is_navigating = False
                break

            target_id = self.route[self.route_index]

            nav = self.calculate_navigation_vector(target_id)

            # robot_missing
            if nav[0] == 'robot_missing':
                # si robot no se ve, esperar
                time.sleep(dt)
                continue

            # target not visible -> esperar hasta que vuelva a aparecer
            if nav[0] == 'target_missing':
                # mantener parada mientras no vemos el objetivo
                self.publish_twist(0.0, 0.0)
                # opcional log esporádico
                if not hasattr(self, '_missing_log_counter'):
                    self._missing_log_counter = 0
                self._missing_log_counter += 1
                if self._missing_log_counter % 20 == 0:
                    self.get_logger().info(f'Waiting: target ArUco {target_id} not visible yet')
                time.sleep(dt)
                continue

            angle_error, distance = nav

            # Llegada por distancia cercana (por redundancia)
            if distance < self.distance_threshold_px:
                self.publish_twist(0.0, 0.0)
                self.get_logger().info(f'Arrived at ArUco {target_id} (distance threshold)')
                self.route_index += 1
                time.sleep(0.5)
                continue

            # Fase de giro
            if abs(angle_error) > self.angle_tolerance_deg:
                angular_z = -angle_error * self.turn_kp
                # limitar
                angular_z = max(min(angular_z, self.turn_speed_rad), -self.turn_speed_rad)
                self.publish_twist(0.0, angular_z)
            else:
                # Avanzar manteniendo pequeña corrección angular
                angular_z = -angle_error * self.turn_kp * 0.6
                self.publish_twist(self.forward_speed, angular_z)

            time.sleep(dt)

        # A la salida, asegurar robot parado
        self.publish_twist(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousArucoRoute()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        # pequeña pausa para terminar hilos
        time.sleep(0.2)
        node.publish_twist(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
