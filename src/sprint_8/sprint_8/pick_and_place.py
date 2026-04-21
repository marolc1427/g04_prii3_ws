#!/usr/bin/env python3

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class ArmArucoController(Node):
    def __init__(self):
        super().__init__('arm_aruco_controller')

        # --- POSICIONES ---
        self.joint_names = ['Junta1', 'Junta2', 'Junta3']
        self.pose_garfio = [0.0, 1.0, 2.0] 
        self.pose_recogida = [1.25, 0.75, 1.0]
        
        self.declare_parameter('tolerance', 20.0)
        self.tolerance = self.get_parameter('tolerance').value

        # Estado: -1: Inicial | 0: Listo | 1: Bajando | 2: En Garfio con pieza | 3: Soltando
        self.state = -1 

        # --- PUBLICADORES Y SUSCRIPTORES ---
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.ventosa_pub = self.create_publisher(Bool, '/ventosa_cmd', 10)
        
        self.aruco_sub = self.create_subscription(String, '/detection/pieza_aruco', self.aruco_callback, 10)

        self.get_logger().info("Iniciando controlador con Ventosa. Esperando conexión...")
        self.init_timer = self.create_timer(1.0, self.check_connection_and_start)

    def control_ventosa(self, activar: bool):
        """Envía el comando a la ventosa."""
        msg = Bool()
        msg.data = activar
        self.ventosa_pub.publish(msg)
        status = "ACTIVADA" if activar else "DESACTIVADA"
        self.get_logger().info(f"Ventosa {status}")

    def move_to_pose(self, positions, time_sec=2):
        """Envía el comando de movimiento al brazo."""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=time_sec)
        msg.points.append(point)
        self.arm_pub.publish(msg)

    def check_connection_and_start(self):
        if self.arm_pub.get_subscription_count() > 0:
            self.init_timer.cancel()
            self.move_to_pose(self.pose_garfio)
            self.control_ventosa(False) # Empezar con ventosa apagada
            self.create_timer(3.0, self.set_ready)
        else:
            self.get_logger().warn("Esperando al brazo...", once=True)

    def set_ready(self):
        # Esta función puede ser llamada por timers, cancelamos si es necesario
        self.state = 0
        self.get_logger().info("Listo. Esperando ArUco centrado...")

    def aruco_callback(self, msg):
        if self.state != 0:
            return 

        try:
            data = json.loads(msg.data)
            if abs(data['error_x']) <= self.tolerance and abs(data['error_y']) <= self.tolerance:
                self.state = 1 # Bloqueamos nuevas detecciones
                self.get_logger().info("¡Centrado! Iniciando ciclo de recogida.")
                self.step_1_bajar_y_succionar()
        except:
            pass

    # --- FLUJO DE LA SECUENCIA ---

    def step_1_bajar_y_succionar(self):
        self.get_logger().info("1. Bajando a por la pieza...")
        self.move_to_pose(self.pose_recogida)
        # Esperamos a que llegue (2s) + un margen para succionar
        self.timer_step = self.create_timer(2.5, self.step_2_activar_succion)

    def step_2_activar_succion(self):
        self.timer_step.cancel()
        self.control_ventosa(True)
        # Tiempo para que la ventosa haga vacío
        self.timer_step = self.create_timer(1.0, self.step_3_subir_a_garfio)

    def step_3_subir_a_garfio(self):
        self.timer_step.cancel()
        self.get_logger().info("2. Pieza cogida. Volviendo a pose GARFIO...")
        self.move_to_pose(self.pose_garfio)
        # Esperamos a que llegue
        self.timer_step = self.create_timer(3.0, self.step_4_volver_a_recogida)

    def step_4_volver_a_recogida(self):
        self.timer_step.cancel()
        self.get_logger().info("3. Bajando de nuevo para soltar...")
        self.move_to_pose(self.pose_recogida)
        # Esperamos a que llegue
        self.timer_step = self.create_timer(2.5, self.step_5_soltar_pieza)

    def step_5_soltar_pieza(self):
        self.timer_step.cancel()
        self.control_ventosa(False)
        # Tiempo para que suelte la pieza físicamente
        self.timer_step = self.create_timer(1.0, self.step_6_finalizar)

    def step_6_finalizar(self):
        self.timer_step.cancel()
        self.get_logger().info("4. Ciclo completo. Regresando a pose GARFIO para esperar...")
        self.move_to_pose(self.pose_garfio)
        # Esperar a estar arriba antes de permitir otra detección
        self.timer_step = self.create_timer(3.0, self.finish_reset)

    def finish_reset(self):
        self.timer_step.cancel()
        self.state = 0
        self.get_logger().info("Sistema listo de nuevo.")

def main(args=None):
    rclpy.init(args=args)
    node = ArmArucoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()