import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
import threading
import time
import math


class ArucoGoTo(Node):
    """Nodo que usa ArUco 8 como referencia (0,0) para mapear posiciones de todos los AruCos
    y navegar el robot a objetivos especificados por coordenadas.
    """

    def __init__(self):
        super().__init__('aruco_go_to')
        # Publicador de velocidad
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Suscriptor a detecciones ArUco (JSON string)
        self.detections_sub = self.create_subscription(String, '/overhead_camera/aruco_detections', self.detections_callback, 10)

        # ArUco de referencia (origen del sistema de coordenadas)
        self.reference_aruco_id = 8
        # ArUco ID del robot (siempre visible, usado para localización)
        self.robot_aruco_id = 3
        
        # Mapa de AruCos: {id: {'x': float, 'y': float, 'px': float, 'py': float}}
        # Coordenadas relativas al ArUco 8
        self.aruco_map = {}
        # Posición de referencia (ArUco 8) en píxeles
        self.reference_position = None
        
        self.lock = threading.Lock()

        # Parámetros de control
        self.forward_speed = 0.12  # m/s
        self.turn_speed_rad = 0.4  # rad/s (máxima velocidad angular)
        self.turn_proportional_gain = 0.01  # Ganancia proporcional para control suave
        
        # Estado de navegación
        self.target_id = None  # ID del ArUco objetivo
        self.is_navigating = False  # Si está en proceso de navegación
        self.navigation_stage = None  # 'rotating' o 'moving_forward'
        self.angle_tolerance_deg = 5.0  # Tolerancia de ángulo antes de avanzar
        self.distance_threshold_px = 30.0  # Distancia mínima para considerar llegada
        self.rotation_start_time = None  # Para timeout de rotación
        self.navigation_complete_flag = False  # Flag para saber cuando termina navegación

        # Hilos para entrada por terminal
        self.running = True
        self.input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.input_thread.start()
        self.control_thread.start()

    def publish_twist(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def detections_callback(self, msg: String):
        """Callback que recibe detecciones JSON y actualiza el mapa de AruCos.
        
        Calcula coordenadas relativas usando ArUco 8 como origen (0,0).
        """
        try:
            arr = json.loads(msg.data)
        except Exception:
            return

        # Buscar ArUco de referencia (id 8)
        ref_aruco = None
        all_detections = []
        
        for item in arr:
            try:
                aruco_id = int(item.get('id'))
                px = float(item.get('px', 0.0))
                py = float(item.get('py', 0.0))
                orientation = float(item.get('orientation', 0.0))
                
                all_detections.append({
                    'id': aruco_id,
                    'px': px,
                    'py': py,
                    'orientation': orientation
                })
                
                if aruco_id == self.reference_aruco_id:
                    ref_aruco = {'px': px, 'py': py}
            except Exception:
                continue
        
        if ref_aruco is None:
            # No se ve el ArUco de referencia, no podemos calcular coordenadas relativas
            return
        
        # Actualizar mapa con coordenadas relativas al ArUco 8
        new_map = {}
        for det in all_detections:
            aruco_id = det['id']
            # Calcular coordenadas relativas (en píxeles por ahora)
            # x positivo a la derecha, y positivo hacia abajo (sistema de imagen)
            rel_x = det['px'] - ref_aruco['px']
            rel_y = det['py'] - ref_aruco['py']
            
            new_map[aruco_id] = {
                'x': rel_x,
                'y': rel_y,
                'px': det['px'],
                'py': det['py'],
                'orientation': det['orientation']
            }
        
        with self.lock:
            self.reference_position = ref_aruco
            self.aruco_map = new_map

    def print_aruco_map(self):
        """Imprime el mapa de AruCos con sus coordenadas relativas al ArUco 8."""
        with self.lock:
            map_snapshot = dict(self.aruco_map)
        
        if not map_snapshot:
            print("\n=== ArUco Map (no detections yet) ===\n")
            return
        
        print("\n" + "="*60)
        print(f"ArUco Map (reference: ArUco {self.reference_aruco_id} at origin 0,0)")
        print("="*60)
        print(f"{'ID':<6} {'X (px)':<12} {'Y (px)':<12} {'Orientation':<15}")
        print("-"*60)
        
        # Ordenar por ID para visualización consistente
        for aruco_id in sorted(map_snapshot.keys()):
            data = map_snapshot[aruco_id]
            x = data['x']
            y = data['y']
            orientation = data['orientation']
            
            # Marcar ArUco especiales
            marker = ""
            if aruco_id == self.reference_aruco_id:
                marker = " [REF]"
            elif aruco_id == self.robot_aruco_id:
                marker = " [ROBOT]"
            
            print(f"{aruco_id:<6} {x:<12.2f} {y:<12.2f} {orientation:<15.2f}{marker}")
        
        print("="*60 + "\n")

    def calculate_navigation_vector(self, target_id):
        """Calcula vector y ángulo necesario para navegar desde el robot (ArUco 3) al objetivo.
        
        Convierte coordenadas de imagen a coordenadas del mundo:
        - Invertir X: x_mundo = -x_imagen (eje X de ArUco es opuesto)
        - Invertir Y: y_mundo = -y_imagen (Y+ hacia abajo en imagen, hacia arriba en mundo)
        
        Retorna: 
        - (angle_deg, distance_px) si ambos AruCos están visibles
        - ('robot_missing', None) si el robot (ArUco 3) no está visible
        - (None, None) si el objetivo no está visible (llegada)
        """
        with self.lock:
            robot_in_map = self.robot_aruco_id in self.aruco_map
            target_in_map = target_id in self.aruco_map
            
            # Si falta el objetivo: llegamos (lo tapamos)
            if not target_in_map:
                return None, None
            
            # Si falta el robot: temporal (se movió), mantener último comando
            if not robot_in_map:
                return 'robot_missing', None
            
            robot_data = self.aruco_map[self.robot_aruco_id]
            target_data = self.aruco_map[target_id]
        
        # Posiciones relativas al origen (ArUco 8) en coordenadas de imagen
        robot_x_img = robot_data['x']
        robot_y_img = robot_data['y']
        target_x_img = target_data['x']
        target_y_img = target_data['y']
        
        # Convertir a coordenadas del mundo:
        # - Invertir X (eje X de ArUco es opuesto al del robot)
        # - Invertir Y (y+ hacia abajo en imagen, hacia arriba en mundo)
        robot_x = -robot_x_img
        robot_y = -robot_y_img
        target_x = -target_x_img
        target_y = -target_y_img
        
        # Vector desde robot a objetivo en coordenadas del mundo
        delta_x = target_x - robot_x
        delta_y = target_y - robot_y
        
        # Distancia euclidiana
        distance = math.sqrt(delta_x**2 + delta_y**2)
        
        # Ángulo del objetivo en coordenadas del mundo (0° = derecha/+X, 90° = arriba/+Y)
        target_angle_rad = math.atan2(delta_y, delta_x)
        target_angle_deg = math.degrees(target_angle_rad)
        
        # Orientación del robot (en grados, según datos del ArUco)
        robot_orientation = robot_data['orientation']
        
        # Ángulo que el robot necesita girar para apuntar al objetivo
        angle_error_deg = target_angle_deg - robot_orientation
        
        # Normalizar a rango [-180, 180]
        while angle_error_deg > 180:
            angle_error_deg -= 360
        while angle_error_deg < -180:
            angle_error_deg += 360
        
        return angle_error_deg, distance

    def _control_loop(self):
        """Loop de control que ejecuta navegación hacia el objetivo."""
        rate_hz = 10.0
        sleep_dt = 1.0 / rate_hz
        
        while self.running:
            if not self.is_navigating or self.target_id is None:
                # No hay objetivo: parar
                self.publish_twist(0.0, 0.0)
                time.sleep(sleep_dt)
                continue
            
            # Calcular vector de navegación
            angle_error, distance = self.calculate_navigation_vector(self.target_id)
            
            # Caso 1: Robot (ArUco 3) temporalmente no visible → mantener último comando
            if angle_error == 'robot_missing':
                # El robot se está moviendo y momentáneamente no se ve
                # Mantener la velocidad actual (no detener)
                time.sleep(sleep_dt)
                continue
            
            # Caso 2: Objetivo desapareció del mapa (robot lo tapó) → llegamos
            if angle_error is None:
                self.publish_twist(0.0, 0.0)
                self.get_logger().info(f'✓ Arrived at ArUco {self.target_id} (target lost - covered by robot)')
                with self.lock:
                    self.is_navigating = False
                    self.navigation_complete_flag = True
                    self.target_id = None
                time.sleep(sleep_dt)
                continue
            
            # Llegada: distancia < threshold
            if distance < self.distance_threshold_px:
                self.publish_twist(0.0, 0.0)
                self.get_logger().info(f'✓ Arrived at ArUco {self.target_id}')
                with self.lock:
                    self.is_navigating = False
                    self.navigation_complete_flag = True
                    self.target_id = None
                time.sleep(sleep_dt)
                continue
            
            # Etapa 1: Girar hacia el objetivo
            if abs(angle_error) > self.angle_tolerance_deg:
                # Control proporcional: velocidad angular basada en el error
                # Esto evita oscilaciones (zig-zag) permitiendo giros suaves
                angular_z = -angle_error * self.turn_proportional_gain
                # Limitar a velocidad máxima
                if angular_z > self.turn_speed_rad:
                    angular_z = self.turn_speed_rad
                elif angular_z < -self.turn_speed_rad:
                    angular_z = -self.turn_speed_rad
                
                self.publish_twist(0.0, angular_z)
                
                if self.rotation_start_time is None:
                    self.rotation_start_time = time.time()
                
                # Log cada ~2 segundos
                if not hasattr(self, '_debug_counter'):
                    self._debug_counter = 0
                self._debug_counter += 1
                if self._debug_counter % 20 == 0:
                    self.get_logger().info(f'Rotating: angle_error={angle_error:.1f}°, angular_z={angular_z:.3f}, distance={distance:.1f}px')
            else:
                # Etapa 2: Avanzar hacia el objetivo (con corrección angular leve)
                self.rotation_start_time = None
                # Mantener alineación mientras avanza (control proporcional pequeño)
                angular_z = -angle_error * self.turn_proportional_gain * 0.5
                self.publish_twist(self.forward_speed, angular_z)
                
                if not hasattr(self, '_debug_counter'):
                    self._debug_counter = 0
                self._debug_counter += 1
                if self._debug_counter % 20 == 0:
                    self.get_logger().info(f'Moving: distance={distance:.1f}px, angle_error={angle_error:.1f}°, angular_z={angular_z:.3f}')
            
            time.sleep(sleep_dt)

    def _input_loop(self):
        """Muestra mapa de AruCos y permite selección continua de objetivo para navegación."""
        # Esperar a recibir el primer mensaje del topic
        print("Waiting for ArUco detections...")
        while self.running:
            time.sleep(0.5)
            with self.lock:
                if len(self.aruco_map) > 0:
                    break
        
        show_menu = True
        last_status_printed = False
        
        while self.running:
            # Mostrar menú si no estamos navegando o si terminamos una navegación
            with self.lock:
                is_nav = self.is_navigating
                nav_complete = self.navigation_complete_flag
            
            if not is_nav and nav_complete:
                # Acabamos de terminar una navegación, mostrar menú
                show_menu = True
                with self.lock:
                    self.navigation_complete_flag = False
            
            if show_menu:
                # Mostrar mapa actualizado de AruCos
                self.print_aruco_map()
                
                print("Commands:")
                print("  - Enter ArUco ID to navigate to that marker")
                print("  - 'map' to refresh the map")
                print("  - 'stop' to cancel navigation")
                print("  - 'quit' to exit")
                
                show_menu = False
                last_status_printed = False
            
            try:
                # Input con timeout (non-blocking)
                line = input("\nEnter command: ").strip() if not is_nav else ""
            except EOFError:
                break
            except KeyboardInterrupt:
                break
            
            if not line:
                if is_nav and not last_status_printed:
                    # Mostrar estado de navegación una vez
                    with self.lock:
                        target = self.target_id
                    print(f"Navigating to ArUco {target}... (enter 'stop' to cancel)")
                    last_status_printed = True
                time.sleep(0.2)
                continue
            
            if line.lower() == 'quit':
                self.get_logger().info('User requested quit')
                self.running = False
                break
            
            if line.lower() == 'stop':
                with self.lock:
                    self.is_navigating = False
                    self.target_id = None
                self.publish_twist(0.0, 0.0)
                print("Navigation stopped.")
                show_menu = True
                continue
            
            if line.lower() == 'map':
                show_menu = True
                continue
            
            # Intentar parsear como ID de ArUco
            try:
                target_id = int(line)
            except ValueError:
                print(f"Invalid command: '{line}'")
                continue
            
            # Validar que el ArUco existe en el mapa
            with self.lock:
                if target_id not in self.aruco_map:
                    print(f"ArUco {target_id} not found in current map")
                    continue
                
                if target_id == self.robot_aruco_id:
                    print(f"Cannot target ArUco {target_id} (robot marker)")
                    continue
                
                target_data = self.aruco_map[target_id]
            
            # Iniciar navegación
            with self.lock:
                self.target_id = target_id
                self.is_navigating = True
                self.rotation_start_time = None
            
            print(f"\n→ Starting navigation to ArUco {target_id} at ({target_data['x']:.1f}, {target_data['y']:.1f})")
            print("Navigating... (enter 'stop' to cancel)\n")
            last_status_printed = False





def main(args=None):
    rclpy.init(args=args)
    node = ArucoGoTo()
    try:
        node.get_logger().info('ArUco navigation started. Mapping AruCos relative to ID 8...')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Request threads to stop
        node.running = False
        # small pause to let threads exit
        time.sleep(0.2)
        node.publish_twist(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

