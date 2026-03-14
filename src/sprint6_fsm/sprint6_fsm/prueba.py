#!/usr/bin/env python3
import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from transitions import Machine, State


@dataclass
class LidarFrontCheck:
    degrees: float = 15.0          # ventana frontal +/- grados
    threshold_m: float = 0.20      # obstáculo si min_dist < threshold
    min_valid_m: float = 0.02      # filtra lecturas raras muy cercanas


class Sprint6FsmLidar(Node):
    """
    FSM:
      - avanzar
      - paro_emergencia
      - girar_derecha_90
      - llegada_objetivo (final)

    Objetivo:
      - Si avanza 3s seguidos sin obstáculo => llegada_objetivo
      - Si detecta obstáculo mientras avanza => paro_emergencia => girar 90º => volver a avanzar
    """

    def __init__(self):
        super().__init__("sprint6_fsm_lidar")

        # ---------------- ROS I/O ----------------
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)

        # ---------------- Parámetros de comportamiento ----------------
        self.check = LidarFrontCheck(degrees=15.0, threshold_m=0.20)
        self.forward_v = 0.12          # m/s
        self.turn_wz = -0.6            # rad/s  (NEGATIVO suele ser derecha en ROS: ajusta si te gira al revés)
        self.goal_clear_s = 3.0        # segundos sin obstáculo para "llegada"
        self.emergency_hold_s = 0.3    # cuánto mantener paro antes de girar
        self.turn_90_rad = math.pi/2   # 90 grados

        # ---------------- Estado LiDAR ----------------
        self.obstacle = False
        self.last_min_front = float("inf")
        self.scan_ready = False
        self._last_obstacle_print = None
        self._last_scan_ready_print = False

        # ---------------- Timers / tiempos ----------------
        self.loop_hz = 20.0
        self.dt = 1.0 / self.loop_hz
        self.timer = self.create_timer(self.dt, self.loop)

        self.clear_start_time = None   # inicio de tramo libre (solo en avanzar)
        self.state_enter_time = self.get_clock().now()

        # ---------------- FSM (Transitions) ----------------
        states = [
            State(name="avanzar"),
            State(name="paro_emergencia"),
            State(name="girar_derecha_90"),
            State(name="llegada_objetivo", final=True),
        ]

        transitions = [
            # Desde AVANZAR:
            {"trigger": "detecta_obstaculo", "source": "avanzar", "dest": "paro_emergencia"},
            {"trigger": "objetivo_logrado",  "source": "avanzar", "dest": "llegada_objetivo"},

            # Desde PARO:
            {"trigger": "paro_listo", "source": "paro_emergencia", "dest": "girar_derecha_90"},

            # Desde GIRAR:
            {"trigger": "giro_completado", "source": "girar_derecha_90", "dest": "avanzar"},
        ]

        self.machine = Machine(
            model=self,
            states=states,
            transitions=transitions,
            initial="avanzar",
            ignore_invalid_triggers=True,  # evita errores si llega un trigger cuando no toca
        )

        self.get_logger().info("Sprint6 FSM LiDAR listo. Estado inicial: avanzar")
        print("[INIT] Sprint6 FSM LiDAR listo. Estado inicial: avanzar")

        # Velocidades preconstruidas
        self.tw_stop = Twist()
        self.tw_fwd = Twist()
        self.tw_fwd.linear.x = float(self.forward_v)
        self.tw_turn = Twist()
        self.tw_turn.angular.z = float(self.turn_wz)

    # ---------------- Callbacks de estados (Transitions crea on_enter_<estado>) ----------------
    def on_enter_avanzar(self):
        self.state_enter_time = self.get_clock().now()
        self.clear_start_time = None  # se reinicia y se empieza a contar cuando esté realmente libre
        self.get_logger().info("Estado: AVANZAR")
        print("[FSM] Estado -> AVANZAR")

    def on_enter_paro_emergencia(self):
        self.state_enter_time = self.get_clock().now()
        self.clear_start_time = None
        self.get_logger().warn(f"Estado: PARO_EMERGENCIA (min_front={self.last_min_front:.2f} m)")
        print(f"[FSM] Estado -> PARO_EMERGENCIA (min_front={self.last_min_front:.2f} m)")

    def on_enter_girar_derecha_90(self):
        self.state_enter_time = self.get_clock().now()
        self.get_logger().info("Estado: GIRAR_DERECHA_90")
        print("[FSM] Estado -> GIRAR_DERECHA_90")

    def on_enter_llegada_objetivo(self):
        self.state_enter_time = self.get_clock().now()
        self.cmd_pub.publish(self.tw_stop)
        self.get_logger().info("He llegado al objetivo ✅ (3s seguidos sin obstáculo)")
        print("[FSM] Estado -> LLEGADA_OBJETIVO")
        print("[OK] He llegado al objetivo (3s seguidos sin obstaculo)")

    # ---------------- LiDAR ----------------
    def scan_callback(self, msg: LaserScan):
        self.scan_ready = True

        if not self._last_scan_ready_print:
            print("[LIDAR] Primer scan recibido, se activa la logica FSM")
            self._last_scan_ready_print = True

        if not msg.ranges:
            self.obstacle = False
            self.last_min_front = float("inf")
            return

        # Cuántos índices corresponden a +/- X grados
        try:
            indices_per_degree = 1.0 / math.degrees(msg.angle_increment)
            k = max(1, int(self.check.degrees * indices_per_degree))
        except ZeroDivisionError:
            k = 10

        # zona frontal: primeros k y últimos k
        front = list(msg.ranges[0:k]) + list(msg.ranges[-k:])

        valid = []
        for d in front:
            if math.isinf(d) or math.isnan(d):
                continue
            if d < msg.range_min or d > msg.range_max:
                continue
            if d < self.check.min_valid_m:
                continue
            valid.append(d)

        if not valid:
            self.last_min_front = float("inf")
            self.obstacle = False
        else:
            self.last_min_front = float(min(valid))
            self.obstacle = (self.last_min_front < self.check.threshold_m)

        # Imprime solo cuando cambia la condicion de obstaculo para no saturar.
        if self._last_obstacle_print is None or self._last_obstacle_print != self.obstacle:
            if self.obstacle:
                print(
                    f"[LIDAR] Obstaculo detectado: min_front={self.last_min_front:.2f} m "
                    f"< threshold={self.check.threshold_m:.2f} m"
                )
            else:
                if math.isinf(self.last_min_front):
                    print("[LIDAR] Zona frontal libre (sin medidas validas cercanas)")
                else:
                    print(
                        f"[LIDAR] Zona frontal libre: min_front={self.last_min_front:.2f} m "
                        f">= threshold={self.check.threshold_m:.2f} m"
                    )
            self._last_obstacle_print = self.obstacle

    # ---------------- Bucle principal ----------------
    def loop(self):
        # Si no hay LiDAR aún, por seguridad paramos (y no avanzamos)
        if not self.scan_ready:
            self.cmd_pub.publish(self.tw_stop)
            return

        now = self.get_clock().now()
        t_in_state = (now - self.state_enter_time).nanoseconds / 1e9

        # --- Estado AVANZAR ---
        if self.state == "avanzar":
            if self.obstacle:
                self.cmd_pub.publish(self.tw_stop)
                print("[LOOP] AVANZAR -> detecta_obstaculo")
                self.detecta_obstaculo()
                return

            # Libre: avanzar y contar tiempo libre continuo
            self.cmd_pub.publish(self.tw_fwd)

            if self.clear_start_time is None:
                self.clear_start_time = now

            t_clear = (now - self.clear_start_time).nanoseconds / 1e9
            if t_clear >= self.goal_clear_s:
                print(f"[LOOP] Objetivo logrado: {t_clear:.2f}s libres continuos")
                self.objetivo_logrado()
                return

        # --- Estado PARO_EMERGENCIA ---
        elif self.state == "paro_emergencia":
            self.cmd_pub.publish(self.tw_stop)
            if t_in_state >= self.emergency_hold_s:
                print(f"[LOOP] PARO_EMERGENCIA completado tras {t_in_state:.2f}s -> girar")
                self.paro_listo()
                return

        # --- Estado GIRAR 90º ---
        elif self.state == "girar_derecha_90":
            # giro por tiempo: tiempo = angulo / |wz|
            wz = float(self.turn_wz)
            if abs(wz) < 1e-6:
                self.cmd_pub.publish(self.tw_stop)
                self.get_logger().error("turn_wz es 0, no puedo girar.")
                print("[ERROR] turn_wz es 0, no puedo girar")
                return

            t_needed = self.turn_90_rad / abs(wz)

            if t_in_state < t_needed:
                self.cmd_pub.publish(self.tw_turn)
            else:
                self.cmd_pub.publish(self.tw_stop)
                print(f"[LOOP] Giro completado en {t_in_state:.2f}s (necesario {t_needed:.2f}s)")
                self.giro_completado()
                return

        # --- Estado final ---
        elif self.state == "llegada_objetivo":
            self.cmd_pub.publish(self.tw_stop)
            return


def main(args=None):
    rclpy.init(args=args)
    node = Sprint6FsmLidar()
    print("[MAIN] Nodo iniciado. Pulsa Ctrl+C para salir")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("[MAIN] Interrupcion por teclado (Ctrl+C)")
    finally:
        print("[MAIN] Parando robot y cerrando nodo")
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()