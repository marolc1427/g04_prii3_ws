#!/usr/bin/env python3

import json
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# =================================================================
# VARIABLES DE CONFIGURACIÓN
# =================================================================
PICKUP_X = 74.75
PICKUP_Y = 169.25
PICKUP_THETA = math.radians(-156.52) 

DROP_X = 406.5
DROP_Y = 176.75
DROP_THETA = math.radians(-80.54) 

# --- AJUSTES DE PRECISIÓN MECÁNICA ---
TOLERANCIA_DIST_BASE = 30.0    
TOLERANCIA_ANGULO_BASE = 0.15   # ~8.5 grados
DISTANCIA_ZONA_LENTA = 90.0   
VEL_MAX = 15                   
VEL_MIN = 10
# CRÍTICA: Subimos a 12 porque 8 es una "orden fantasma" que no mueve el robot.
VEL_GIRO_FINO = 12             
# =================================================================

class EurobotBrainNode(Node):
    def __init__(self):
        super().__init__('eurobot_brain_node')

        self.joint_names = ['Junta1', 'Junta2', 'Junta3']
        self.pose_garfio = [0.0, 1.0, 2.0] 
        self.pose_recogida = [1.25, 0.75, 1.0]

        self.state = 0
        self.robot_pose = None
        self.timer_step = None

        self.motor_pub = self.create_publisher(Int32MultiArray, '/cmd_motores', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.ventosa_pub = self.create_publisher(Bool, '/cmd_ventosa', 10)
        
        self.aruco_base_sub = self.create_subscription(String, '/detection/aruco_3', self.base_camera_callback, 10)

        self.control_timer = self.create_timer(0.1, self.main_fsm_loop)
        self.get_logger().info("MODO PRECISIÓN V2: Corregido umbral de potencia mínima.")

    def base_camera_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.robot_pose = {
                'x': float(data['x']),
                'y': float(data['y']),
                'theta': math.radians(float(data.get('angle', 0.0)))
            }
        except Exception: pass

    def _normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def main_fsm_loop(self):
        if not self.robot_pose: return

        if self.state == 0: 
            if self.navigate_to(PICKUP_X, PICKUP_Y, PICKUP_THETA):
                self.get_logger().info("🎯 Posición y ángulo clavados. Recogiendo...")
                self.state = 2
                self.secuencia_recogida()

        elif self.state == 3: 
            if self.navigate_to(DROP_X, DROP_Y, DROP_THETA):
                self.get_logger().info("🎯 Posición y ángulo de dejada listos.")
                self.state = 4
                self.secuencia_dejada()

        elif self.state in [2, 4, 5]:
            self.stop_base()

    def navigate_to(self, tx, ty, t_theta):
        dx = tx - self.robot_pose['x']
        dy = ty - self.robot_pose['y']
        dist = math.hypot(dx, dy)
        
        target_heading = math.atan2(dy, dx)
        error_angle_nav = self._normalize_angle(target_heading - self.robot_pose['theta'])
        error_angle_final = self._normalize_angle(t_theta - self.robot_pose['theta'])

        motor_msg = Int32MultiArray()

        # 1. AJUSTE FINO (Prioridad absoluta al ángulo cuando estamos cerca)
        if dist < TOLERANCIA_DIST_BASE:
            if abs(error_angle_final) > TOLERANCIA_ANGULO_BASE:
                self.get_logger().info(f"Ajustando ángulo... Error: {math.degrees(error_angle_final):.1f}º", throttle_duration_sec=1.0)
                
                # Usamos VEL_GIRO_FINO (12) para asegurar que los motores venzan la fricción
                motor_msg.data = [-VEL_GIRO_FINO, VEL_GIRO_FINO] if error_angle_final > 0 else [VEL_GIRO_FINO, -VEL_GIRO_FINO]
                self.motor_pub.publish(motor_msg)
                return False 
            else:
                self.stop_base()
                return True

        # 2. NAVEGACIÓN GENERAL
        current_vel = VEL_MAX if dist > DISTANCIA_ZONA_LENTA else VEL_MIN

        if abs(error_angle_nav) > math.radians(25):
            motor_msg.data = [-current_vel, current_vel] if error_angle_nav > 0 else [current_vel, -current_vel]
        else:
            motor_msg.data = [current_vel, current_vel]
        
        self.motor_pub.publish(motor_msg)
        return False

    # --- SECUENCIAS (Sin cambios) ---
    def secuencia_recogida(self):
        self.move_arm(self.pose_recogida)
        self.timer_step = self.create_timer(2.5, self.recogida_succion)

    def recogida_succion(self):
        self.timer_step.cancel()
        self.control_ventosa(True)
        self.timer_step = self.create_timer(1.0, self.recogida_subir)

    def recogida_subir(self):
        self.timer_step.cancel()
        self.move_arm(self.pose_garfio)
        self.timer_step = self.create_timer(3.0, lambda: self.set_state(3))

    def set_state(self, new_state):
        if self.timer_step: self.timer_step.cancel()
        self.state = new_state

    def secuencia_dejada(self):
        self.move_arm(self.pose_recogida)
        self.timer_step = self.create_timer(2.5, self.soltar_ventosa)

    def soltar_ventosa(self):
        self.timer_step.cancel()
        self.control_ventosa(False)
        self.timer_step = self.create_timer(1.0, self.mision_finalizada)

    def mision_finalizada(self):
        self.timer_step.cancel()
        self.move_arm(self.pose_garfio)
        self.state = 5
        self.get_logger().info("✅ Misión completada.")

    def control_ventosa(self, activar: bool):
        msg = Bool(); msg.data = activar
        self.ventosa_pub.publish(msg)

    def move_arm(self, positions):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=2)
        msg.points.append(point)
        self.arm_pub.publish(msg)

    def stop_base(self):
        m = Int32MultiArray(); m.data = [0, 0]
        self.motor_pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = EurobotBrainNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: 
        node.stop_base()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()   