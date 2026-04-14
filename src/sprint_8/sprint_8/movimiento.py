import json
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class MovimientoTest(Node):
	def __init__(self):
		super().__init__('movimiento_test')

		# --- Parámetros (configurables por launch/CLI) ---
		self.declare_parameter('robot_id', 3)
		self.declare_parameter('route_ids', '20,21')

		# Tolerancias / control
		self.declare_parameter('distance_tolerance', 35.0)
		self.declare_parameter('kp_linear', 0.0004)
		self.declare_parameter('kp_angular', 0.35)
		self.declare_parameter('max_lin_vel', 0.8)
		self.declare_parameter('max_ang_vel', 0.40)
		self.declare_parameter('turn_in_place_threshold_deg', 20.0)

		# Ajustes de frame/signo (útil cuando el ArUco está girado respecto al “frente” real)
		self.declare_parameter('theta_offset_deg', 0.0)
		self.declare_parameter('invert_angular', True)

		# Temporización
		self.declare_parameter('control_rate_hz', 10.0)
		self.declare_parameter('stale_timeout_sec', 0.7)

		self.robot_id = int(self.get_parameter('robot_id').value)
		self.route_ids = self._parse_ids(self.get_parameter('route_ids').value)
		self.route_ids = [i for i in self.route_ids if i != self.robot_id]
		if not self.route_ids:
			self.get_logger().warning(
				"'route_ids' está vacío o contiene solo el robot_id; usando [1] por defecto."
			)
			self.route_ids = [1]

		self.distance_tolerance = float(self.get_parameter('distance_tolerance').value)
		self.kp_linear = float(self.get_parameter('kp_linear').value)
		self.kp_angular = float(self.get_parameter('kp_angular').value)
		self.max_lin_vel = float(self.get_parameter('max_lin_vel').value)
		self.max_ang_vel = float(self.get_parameter('max_ang_vel').value)
		self.turn_threshold = math.radians(
			float(self.get_parameter('turn_in_place_threshold_deg').value)
		)
		self.theta_offset = math.radians(float(self.get_parameter('theta_offset_deg').value))
		self.invert_angular = bool(self.get_parameter('invert_angular').value)

		self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
		self.stale_timeout_sec = float(self.get_parameter('stale_timeout_sec').value)

		# --- Estado ---
		self.robot_pose = None  # {'x': float, 'y': float, 'theta': float, 'stamp': float}
		self.targets = {}  # id -> {'x','y','stamp'}
		self.route_index = 0
		self.mission_complete = False

		# --- ROS interfaces ---
		self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 30)

		# Robot pose topic
		self.create_subscription(
			String, f'/detection/aruco_{self.robot_id}', self._robot_cb, 10
		)

		# Target topics
		self.target_subs = {}
		for tid in self.route_ids:
			self.target_subs[tid] = self.create_subscription(
				String,
				f'/detection/aruco_{tid}',
				lambda msg, tid=tid: self._target_cb(msg, tid),
				10,
			)

		period = 1.0 / max(self.control_rate_hz, 1e-6)
		self.timer = self.create_timer(period, self.control_loop)

		self.get_logger().info(
			f"MovimientoTest iniciado | robot_id={self.robot_id} | route={self.route_ids} | "
			f"offset={math.degrees(self.theta_offset):.1f}º | invert_angular={self.invert_angular}"
		)

	def _parse_ids(self, s):
		if s is None:
			return []
		if isinstance(s, (list, tuple)):
			return [int(x) for x in s]
		s = str(s).strip()
		if not s:
			return []
		out = []
		for part in s.split(','):
			part = part.strip()
			if not part:
				continue
			out.append(int(part))
		return out

	def _normalize_angle(self, angle):
		while angle > math.pi:
			angle -= 2.0 * math.pi
		while angle < -math.pi:
			angle += 2.0 * math.pi
		return angle

	def _parse_detection_json(self, json_str):
		"""Parsea el JSON publicado por `aruco_detector_node.py`.

		Esperado: {"id": int, "x": float, "y": float, "angle": float}.
		"""
		try:
			data = json.loads(json_str)
		except Exception:
			return None

		# Formato sprint_8
		if 'x' in data and 'y' in data:
			x = float(data['x'])
			y = float(data['y'])
			angle_deg = float(data.get('angle', 0.0))
			theta = math.radians(angle_deg)
			return {'x': x, 'y': y, 'theta': theta}

		# Formato alternativo (por si viene del otro detector)
		if 'px' in data and 'py' in data:
			x = float(data['px'])
			y = float(data['py'])
			theta = math.radians(float(data.get('orientation', 0.0)))
			return {'x': x, 'y': y, 'theta': theta}

		return None

	def _robot_cb(self, msg: String):
		p = self._parse_detection_json(msg.data)
		if not p:
			return
		# Offset opcional del heading
		p['theta'] = self._normalize_angle(p['theta'] + self.theta_offset)
		p['stamp'] = self.get_clock().now().nanoseconds * 1e-9
		self.robot_pose = p

	def _target_cb(self, msg: String, tid: int):
		p = self._parse_detection_json(msg.data)
		if not p:
			return
		now = self.get_clock().now().nanoseconds * 1e-9
		self.targets[tid] = {'x': p['x'], 'y': p['y'], 'stamp': now}

	def _is_stale(self, stamp_sec: float):
		now = self.get_clock().now().nanoseconds * 1e-9
		return (now - float(stamp_sec)) > self.stale_timeout_sec

	def control_loop(self):
		twist = Twist()

		if self.mission_complete:
			self.cmd_pub.publish(twist)
			return

		if not self.robot_pose:
			self.get_logger().warning('Esperando pose del robot...', throttle_duration_sec=2.0)
			self.cmd_pub.publish(twist)
			return
		if self._is_stale(self.robot_pose.get('stamp', 0.0)):
			self.get_logger().warning('Pose del robot desactualizada...', throttle_duration_sec=2.0)
			self.cmd_pub.publish(twist)
			return

		current_target_id = self.route_ids[self.route_index]
		if current_target_id not in self.targets:
			self.get_logger().warning(
				f'Esperando target {current_target_id}...', throttle_duration_sec=2.0
			)
			self.cmd_pub.publish(twist)
			return

		target = self.targets[current_target_id]
		if self._is_stale(target.get('stamp', 0.0)):
			self.get_logger().warning(
				f'Target {current_target_id} desactualizado...', throttle_duration_sec=2.0
			)
			self.cmd_pub.publish(twist)
			return

		# 1) Error en el plano (coordenadas de imagen/warp)
		dx = float(target['x']) - float(self.robot_pose['x'])
		dy = float(target['y']) - float(self.robot_pose['y'])
		distance = math.hypot(dx, dy)
		target_heading = math.atan2(dy, dx)
		heading_error = self._normalize_angle(target_heading - float(self.robot_pose['theta']))

		# 2) Llegada
		if distance < self.distance_tolerance:
			self.get_logger().info(f'!!! LLEGADA A {current_target_id} !!!')
			self.route_index += 1
			if self.route_index >= len(self.route_ids):
				self.mission_complete = True
				self.get_logger().info('Misión terminada.')
			self.cmd_pub.publish(Twist())
			return

		# 3) Control (turn-stop-move)
		sign = -1.0 if self.invert_angular else 1.0
		angular_z = sign * heading_error * self.kp_angular

		if abs(heading_error) > self.turn_threshold:
			linear_x = 0.0
			mode = 'ROTATING'
		else:
			linear_x = distance * self.kp_linear
			mode = 'FORWARD'

		# Saturación
		angular_z = max(min(angular_z, self.max_ang_vel), -self.max_ang_vel)
		linear_x = max(min(linear_x, self.max_lin_vel), -self.max_lin_vel)

		self.get_logger().info(
			f'T:{current_target_id} | {mode} | Dist:{distance:.1f} | '
			f'Err:{math.degrees(heading_error):.1f}º | V:{linear_x:.3f} | W:{angular_z:.3f}',
			throttle_duration_sec=0.25,
		)

		twist.linear.x = float(linear_x)
		twist.angular.z = float(angular_z)
		self.cmd_pub.publish(twist)


def main(args=None):
	rclpy.init(args=args)
	node = MovimientoTest()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.cmd_pub.publish(Twist())
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()