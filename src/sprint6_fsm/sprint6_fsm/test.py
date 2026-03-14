#!/usr/bin/env python3
import json
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from transitions import Machine  # librería pedida por el profe


@dataclass
class Detection:
    px: float
    py: float
    stamp: float  # time.time()


class RobotFSM(Node):
    def __init__(self):
        super().__init__("fsm_sprint6")

        # ---------- Params “tuneables” ----------
        self.image_width = 1080  # según lo que dijiste: 720x1080 (ancho=1080)
        self.center_x = self.image_width / 2.0

        self.stable_hits_required = 3
        self.align_px_tolerance = 20.0
        self.lost_timeout_s = 0.5       # si en 0.5s no llega detección -> “perdido”
        self.scan_timeout_s = 10.0      # si en SCAN pasa mucho sin ver -> RECOVERY
        self.approach_time_s = 3.0      # avanzar 3 segundos

        # velocidades (suaves)
        self.scan_wz = 0.35
        self.align_kp = 0.0025          # w = -Kp * err_px
        self.align_wz_max = 0.6
        self.approach_vx = 0.10
        self.approach_kp = 0.0015
        self.approach_wz_max = 0.35

        # recovery “simple”
        self.recovery_back_s = 0.4
        self.recovery_turn_s = 0.6
        self.recovery_vx = -0.08
        self.recovery_wz = 0.4

        # ---------- ROS I/O ----------
        self.sub = self.create_subscription(
            String,
            "/aruco_detections",
            self.on_detection_msg,
            qos_profile_sensor_data
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ---------- Detection state ----------
        self.last_det: Optional[Detection] = None
        self._stable_hits = 0

        # ---------- Timers / time ----------
        self._state_enter_time = time.time()
        self._last_seen_time = 0.0
        self._scan_start_time = time.time()

        # ---------- FSM (transitions) ----------
        states = ["SCAN", "ALIGN", "APPROACH", "CHECKPOINT", "RECOVERY"]

        transitions = [
            # SCAN -> ALIGN si objetivo estable
            {"trigger": "see_target", "source": "SCAN", "dest": "ALIGN", "conditions": "target_is_stable"},

            # SCAN -> RECOVERY si timeout sin verlo
            {"trigger": "scan_timeout", "source": "SCAN", "dest": "RECOVERY"},

            # ALIGN -> APPROACH si está alineado (estable)
            {"trigger": "aligned", "source": "ALIGN", "dest": "APPROACH", "conditions": "is_aligned"},

            # ALIGN -> RECOVERY si se pierde
            {"trigger": "lost", "source": "ALIGN", "dest": "RECOVERY"},

            # APPROACH -> CHECKPOINT cuando pasan 3s
            {"trigger": "approach_done", "source": "APPROACH", "dest": "CHECKPOINT"},

            # APPROACH -> RECOVERY si se pierde (opcional, pero útil)
            {"trigger": "lost", "source": "APPROACH", "dest": "RECOVERY"},

            # CHECKPOINT -> SCAN (si queréis demo en bucle)
            {"trigger": "restart", "source": "CHECKPOINT", "dest": "SCAN"},

            # RECOVERY -> SCAN siempre al terminar recovery
            {"trigger": "recovered", "source": "RECOVERY", "dest": "SCAN"},
        ]

        self.machine = Machine(
            model=self,
            states=states,
            transitions=transitions,
            initial="SCAN",
        )

        # Timer principal de control (10 Hz)
        self.timer = self.create_timer(0.1, self.control_tick)

        self.get_logger().info("FSM Sprint 6 iniciada en estado SCAN")

    # -------------------- ROS callbacks --------------------
    def on_detection_msg(self, msg: String):
        """Recibe detecciones desde el nodo de cámara.
        Esperado: JSON con formato:
        {
          "width": 1080,
          "height": 720,
          "stamp": {"sec": ..., "nanosec": ...},
          "frame_id": "...",
          "detections": [{"id": 47, "px": ..., "py": ..., "corners": [...]}, ...]
        }
        """
        now = time.time()
        try:
            data = json.loads(msg.data)
        except Exception:
            return  # si llega algo raro, lo ignoramos

        if isinstance(data, dict):
            width = data.get("width")
            if width is not None:
                try:
                    self.image_width = float(width)
                    self.center_x = self.image_width / 2.0
                except Exception:
                    pass

        det = self._extract_target_detection(data, now)
        if det is None:
            return

        self.last_det = det
        self._last_seen_time = now

        # estabilidad: hits seguidos con detección válida
        self._stable_hits = min(self._stable_hits + 1, self.stable_hits_required)

    def _extract_target_detection(self, data, stamp: float) -> Optional[Detection]:
        # Caso 0: payload actual del detector ArUco (usamos solo px/py)
        if isinstance(data, dict) and isinstance(data.get("detections"), list):
            for item in data["detections"]:
                if isinstance(item, dict) and ("px" in item) and ("py" in item):
                    try:
                        return Detection(
                            px=float(item["px"]),
                            py=float(item["py"]),
                            stamp=stamp,
                        )
                    except Exception:
                        continue

        # Caso 1: dict simple (compatibilidad)
        if isinstance(data, dict):
            if ("px" in data) and ("py" in data):
                try:
                    return Detection(
                        px=float(data["px"]),
                        py=float(data["py"]),
                        stamp=stamp,
                    )
                except Exception:
                    return None

        # Caso 2: lista de dicts (compatibilidad)
        if isinstance(data, list):
            for item in data:
                if isinstance(item, dict) and ("px" in item) and ("py" in item):
                    try:
                        return Detection(
                            px=float(item["px"]),
                            py=float(item["py"]),
                            stamp=stamp,
                        )
                    except Exception:
                        continue
        return None

    # -------------------- Conditions (transitions) --------------------
    def target_is_stable(self) -> bool:
        return self._stable_hits >= self.stable_hits_required

    def is_aligned(self) -> bool:
        det = self.last_det
        if det is None:
            return False
        err = det.px - self.center_x
        return abs(err) <= self.align_px_tolerance

    # -------------------- Helpers --------------------
    def _publish_cmd(self, vx: float, wz: float):
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        self.cmd_pub.publish(t)

    def _enter_state(self):
        self._state_enter_time = time.time()

    # -------------------- Main control loop --------------------
    def control_tick(self):
        now = time.time()

        # “Perdido”: hace mucho que no recibimos detección
        target_visible = (now - self._last_seen_time) <= self.lost_timeout_s

        # --- lógica de disparo de triggers (según estado) ---
        if self.state == "SCAN":
            if self.target_is_stable():
                self.see_target()
                self._enter_state()
                return

            if (now - self._scan_start_time) > self.scan_timeout_s:
                self.scan_timeout()
                self._enter_state()
                return

            # acción SCAN
            self._publish_cmd(0.0, self.scan_wz)

        elif self.state == "ALIGN":
            if not target_visible:
                self._stable_hits = 0
                self.lost()
                self._enter_state()
                return

            det = self.last_det
            err = (det.px - self.center_x) if det else 0.0
            wz = -self.align_kp * err
            wz = max(-self.align_wz_max, min(self.align_wz_max, wz))
            self._publish_cmd(0.0, wz)

            if self.is_aligned():
                self.aligned()
                self._enter_state()
                return

        elif self.state == "APPROACH":
            # opcional: si se pierde, recovery
            if not target_visible:
                self._stable_hits = 0
                self.lost()
                self._enter_state()
                return

            elapsed = now - self._state_enter_time
            if elapsed >= self.approach_time_s:
                self.approach_done()
                self._enter_state()
                return

            det = self.last_det
            err = (det.px - self.center_x) if det else 0.0
            wz = -self.approach_kp * err
            wz = max(-self.approach_wz_max, min(self.approach_wz_max, wz))
            self._publish_cmd(self.approach_vx, wz)

        elif self.state == "CHECKPOINT":
            # parar y (por ejemplo) volver a SCAN
            self._publish_cmd(0.0, 0.0)
            # demo: espera 1s y reinicia
            if (now - self._state_enter_time) > 1.0:
                self._stable_hits = 0
                self._scan_start_time = now
                self.restart()
                self._enter_state()

        elif self.state == "RECOVERY":
            # recovery simple secuenciado por tiempo
            elapsed = now - self._state_enter_time

            if elapsed < 0.2:
                self._publish_cmd(0.0, 0.0)
            elif elapsed < (0.2 + self.recovery_back_s):
                self._publish_cmd(self.recovery_vx, 0.0)
            elif elapsed < (0.2 + self.recovery_back_s + self.recovery_turn_s):
                self._publish_cmd(0.0, self.recovery_wz)
            else:
                # fin recovery -> volver a SCAN
                self._publish_cmd(0.0, 0.0)
                self._stable_hits = 0
                self._scan_start_time = now
                self.recovered()
                self._enter_state()


def main(args=None):
    rclpy.init(args=args)
    node = RobotFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()