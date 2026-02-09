import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
import math

class ArucoNavigator(Node):
    def __init__(self):
        super().__init__('aruco_navigator')

        self.robot_id = 3
        self.static_ids = [20, 21, 22, 23]
        
        # --- CONFIGURACIÓN ---
        self.DIST_TOLERANCE = 35.0 
        
        # Ganancias
        self.KP_LINEAR = 0.0004 
        self.KP_ANGULAR = 0.35   

        # Límites
        self.MAX_LIN_VEL = 0.08
        self.MAX_ANG_VEL = 0.3

        self.robot_pose = None
        self.targets = {}
        self.route_index = 0
        self.mission_complete = False

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(String, f'/overhead_camera/aruco_{self.robot_id}', self.robot_cb, 10)

        self.static_subs = {}
        for tid in self.static_ids:
            self.static_subs[tid] = self.create_subscription(
                String, f'/overhead_camera/aruco_{tid}', 
                lambda msg, tid=tid: self.static_aruco_cb(msg, tid), 10
            )

        self.timer = self.create_timer(1.0, self.control_loop)
        self.get_logger().info("Nodo v5 (Offset 180 + Giro Invertido) iniciado.")

    def parse_aruco_json(self, json_str):
        try:
            data = json.loads(json_str)
            return {'x': data['px'], 'y': data['py'], 'theta': math.radians(data['orientation'])}
        except: return None

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def robot_cb(self, msg):
        p = self.parse_aruco_json(msg.data)
        if p: 
            # --- CORRECCIÓN CRÍTICA ---
            # El ArUco está rotado 180 grados respecto al frente del robot.
            # Sumamos PI para alinear el "frente lógico" con el "frente físico".
            p['theta'] = self.normalize_angle(p['theta'] + math.pi)
            self.robot_pose = p

    def static_aruco_cb(self, msg, tid):
        if tid in self.targets: return
        p = self.parse_aruco_json(msg.data)
        if p:
            self.targets[tid] = p
            if tid in self.static_subs:
                self.destroy_subscription(self.static_subs[tid])
                del self.static_subs[tid]

    def control_loop(self):
        twist = Twist()

        if self.mission_complete:
            self.cmd_pub.publish(twist)
            return

        if not self.robot_pose:
            self.get_logger().warning("Esperando robot...", throttle_duration_sec=2)
            return

        current_id = self.static_ids[self.route_index]
        if current_id not in self.targets:
            self.get_logger().warning(f"Esperando target {current_id}...", throttle_duration_sec=2)
            return

        target = self.targets[current_id]
        
        # 1. Calcular Error
        dx = target['x'] - self.robot_pose['x']
        dy = target['y'] - self.robot_pose['y']
        distance = math.sqrt(dx**2 + dy**2)
        target_heading = math.atan2(dy, dx)
        
        heading_error = self.normalize_angle(target_heading - self.robot_pose['theta'])

        # 2. Verificar Llegada
        if distance < self.DIST_TOLERANCE:
            self.get_logger().info(f"!!! LLEGADA A {current_id} !!!")
            self.route_index += 1
            if self.route_index >= len(self.static_ids):
                self.mission_complete = True
                self.get_logger().info("Misión terminada.")
            self.cmd_pub.publish(Twist()) 
            return

        # 3. Control
        # Usamos la lógica v4 que sabemos que reduce el error correctamente
        angular_z = -1.0 * heading_error * self.KP_ANGULAR

        # Lógica Turn-Stop-Move
        if abs(heading_error) > 0.35: # ~20 grados
            linear_x = 0.0
            mode = "ROTATING"
        else:
            linear_x = (distance * self.KP_LINEAR)
            mode = "FORWARD"

        # Saturación
        angular_z = max(min(angular_z, self.MAX_ANG_VEL), -self.MAX_ANG_VEL)
        linear_x = max(min(linear_x, self.MAX_LIN_VEL), -self.MAX_LIN_VEL)

        self.get_logger().info(
            f"T:{current_id} | {mode} | Dist:{distance:.0f} | Err:{math.degrees(heading_error):.0f}º | CmdAng:{angular_z:.3f}"
        )

        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()