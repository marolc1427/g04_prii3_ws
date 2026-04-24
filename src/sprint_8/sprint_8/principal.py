#!/usr/bin/env python3

import json
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# =================================================================
# VARIABLES GLOBALES DE CONFIGURACIÓN (Ajustar según tablero)
# =================================================================
# Posición aproximada de la pieza (Cámara Cenital)
PICKUP_X = 450.0
PICKUP_Y = 300.0
PICKUP_THETA = 1.57  # Radianes (aprox 90 grados)

# Posición de la zona de dejada (Cámara Cenital)
DROP_X = 800.0
DROP_Y = 600.0
DROP_THETA = 0.0     # Radianes

# Tolerancias
TOLERANCIA_DIST_BASE = 15.0  # Píxeles (cenital)
TOLERANCIA_ANGULO_BASE = 0.1 # Radianes
TOLERANCIA_CENTRAD_BRAZO = 20.0 # Píxeles (cámara brazo)
# =================================================================

class EurobotBrainNode(Node):
    def __init__(self):
        super().__init__('eurobot_brain_node')

        # Parámetros del Brazo
        self.joint_names = ['Junta1', 'Junta2', 'Junta3']
        self.pose_garfio = [0.0, 1.0, 2.0] 
        self.pose_recogida = [1.25, 0.75, 1.0]

        # Estados: 0: Viaje a pieza | 1: Centrado Brazo | 2: Recogida | 3: Viaje a dejada | 4: Dejada | 5: Fin
        self.state = 0
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.pose_updated = False
        self.timer_step = None

        # Publicadores
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.ventosa_pub = self.create_publisher(Bool, '/ventosa_cmd', 10)
        
        # Suscriptores
        self.aruco_base_sub = self.create_subscription(String, '/detection/aruco_3', self.base_camera_callback, 10)
        self.aruco_pieza_sub = self.create_subscription(String, '/detection/pieza_aruco', self.arm_camera_callback, 10)

        self.control_timer = self.create_timer(0.1, self.main_fsm_loop)
        self.get_logger().info("Cerebro iniciado. Navegando a posición de recogida...")

    def base_camera_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.robot_pose['x'] = float(data['x'])
            self.robot_pose['y'] = float(data['y'])
            # Conversión necesaria si el detector envía grados
            self.robot_pose['theta'] = math.radians(float(data['angle']))
            self.pose_updated = True
        except: pass

    def arm_camera_callback(self, msg):
        # Esta lógica solo se activa cuando el robot ya está en la zona de recogida
        if self.state != 1: return
        
        try:
            data = json.loads(msg.data)
            err_x = abs(data['error_x'])
            err_y = abs(data['error_y'])
            
            if err_x <= TOLERANCIA_CENTRAD_BRAZO and err_y <= TOLERANCIA_CENTRAD_BRAZO:
                self.get_logger().info("¡Centrado preciso logrado! Iniciando secuencia mecánica.")
                self.state = 2
                self.secuencia_recogida()
        except: pass

    def main_fsm_loop(self):
        if not self.pose_updated: return

        if self.state == 0: # Navegación global a la pieza
            if self.navigate_to(PICKUP_X, PICKUP_Y, PICKUP_THETA):
                self.get_logger().info("En zona. Buscando pieza con cámara del brazo...")
                self.state = 1

        elif self.state == 3: # Navegación a zona de dejada
            if self.navigate_to(DROP_X, DROP_Y, DROP_THETA):
                self.state = 4
                self.secuencia_dejada()

        elif self.state in [1, 2, 4]: # Estados donde la base debe estar estática
            self.stop_base()

    def navigate_to(self, tx, ty, t_theta):
        """Calcula velocidad para llegar a X, Y y orientarse a THETA."""
        dx = tx - self.robot_pose['x']
        dy = ty - self.robot_pose['y']
        dist = math.sqrt(dx**2 + dy**2)
        
        # Error de ángulo hacia el objetivo (para avanzar)
        target_angle = math.atan2(dy, dx)
        error_angle_nav = math.atan2(math.sin(target_angle - self.robot_pose['theta']), 
                                    math.cos(target_angle - self.robot_pose['theta']))
        
        # Error de ángulo final (orientación deseada)
        error_angle_final = math.atan2(math.sin(t_theta - self.robot_pose['theta']), 
                                      math.cos(t_theta - self.robot_pose['theta']))

        twist = Twist()

        # 1. Si está lejos, navega hacia el punto
        if dist > TOLERANCIA_DIST_BASE:
            if abs(error_angle_nav) > 0.3:
                twist.angular.z = 1.2 * error_angle_nav # Giro puro
            else:
                twist.linear.x = 0.2
                twist.angular.z = 0.6 * error_angle_nav # Avance con corrección
        
        # 2. Si está cerca, se orienta al ángulo final
        elif abs(error_angle_final) > TOLERANCIA_ANGULO_BASE:
            twist.angular.z = 0.8 * error_angle_final
            
        # 3. En posición y orientado
        else:
            self.stop_base()
            return True

        self.cmd_vel_pub.publish(twist)
        return False

    # --- SECUENCIAS MECÁNICAS (BRAZO Y VENTOSA) ---

    def secuencia_recogida(self):
        self.get_logger().info("1. Bajando...")
        self.move_arm(self.pose_recogida)
        self.timer_step = self.create_timer(2.5, self.recogida_succion)

    def recogida_succion(self):
        self.timer_step.cancel()
        self.control_ventosa(True)
        self.timer_step = self.create_timer(1.0, self.recogida_subir)

    def recogida_subir(self):
        self.timer_step.cancel()
        self.move_arm(self.pose_garfio)
        self.timer_step = self.create_timer(3.0, self.finalizar_recogida)

    def finalizar_recogida(self):
        self.timer_step.cancel()
        self.state = 3
        self.get_logger().info("Pieza en garfio. Viajando a zona de dejada...")

    def secuencia_dejada(self):
        self.get_logger().info("Bajando para soltar...")
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
        self.get_logger().info("Misión completada con éxito.")

    # --- UTILIDADES ---
    def control_ventosa(self, activar: bool):
        msg = Bool()
        msg.data = activar
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
        self.cmd_vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = EurobotBrainNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()