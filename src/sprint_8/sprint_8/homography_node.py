import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class BoardProcessor(Node):
    def __init__(self):
        super().__init__('board_processor_node')
        
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        
        self.pose_pub = self.create_publisher(Pose2D, '/robot_pose', 10)
        self.image_pub = self.create_publisher(Image, '/image_warped', 10)
        
        self.bridge = CvBridge()
        self.dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dict, self.params)

        # CONFIGURACIÓN DE DIMENSIONES REALES
        self.esc = 5
        # Cambiamos esto según la realidad de tu tablero:
        # Si la distancia 23-22 es el lado largo, 180 es correcto.
        self.w_u, self.h_u = 180 * self.esc, 80 * self.esc
        self.m = 55 * self.esc  
        self.w_tot, self.h_tot = int(self.w_u + 2 * self.m), int(self.h_u + 2 * self.m)

        self.M = None
        self.last_h_update = 0

    def image_callback(self, msg):
        self.get_logger().info("--- Frame recibido ---")
        
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)).apply(gray)
        
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None:
            ids_list = ids.flatten().tolist()
            
            # --- PARTE 1: RECALCULAR HOMOGRAFÍA (Mapeo por ID) ---
            now = time.time()
            ref_ids = [23, 22, 20, 21] # TL, TR, BR, BL
            
            # Buscamos si están los 4 de referencia
            if all(rid in ids_list for rid in ref_ids) and (now - self.last_h_update > 1.0):
                # Extraemos los centros en orden específico
                centers_dict = {ids_list[i]: corners[i][0].mean(axis=0) for i in range(len(ids_list))}
                
                src = np.array([
                    centers_dict[23], # Top-Left
                    centers_dict[22], # Top-Right
                    centers_dict[20], # Bottom-Right
                    centers_dict[21]  # Bottom-Left
                ], dtype="float32")
                
                # Destino con el aspecto correcto
                dst = np.array([
                    [self.m, self.m], 
                    [self.w_u + self.m, self.m], 
                    [self.w_u + self.m, self.h_u + self.m], 
                    [self.m, self.h_u + self.m]
                ], dtype="float32")
                
                self.M, _ = cv2.findHomography(src, dst)
                self.last_h_update = now
                self.get_logger().info("Homografía actualizada correctamente")
            else:
                # Feedback para que sepas por qué no se actualiza
                missing = [rid for rid in ref_ids if rid not in ids_list]
                if missing:
                    self.get_logger().warn(f"Esperando marcadores: {missing}", once=True)

            # --- PARTE 2: PROCESAMIENTO CON MATRIZ EXISTENTE ---
            if self.M is not None:
                out = cv2.warpPerspective(frame, self.M, (self.w_tot, self.h_tot))
                
                # Seguimiento del robot ID 3
                if 3 in ids_list:
                    idx_robot = ids_list.index(3)
                    robot_corners = corners[idx_robot][0]
                    center_px = np.mean(robot_corners, axis=0).reshape(-1, 1, 2)
                    world_pt = cv2.perspectiveTransform(center_px, self.M)[0][0]
                    
                    # Publicar Pose
                    pose = Pose2D()
                    pose.x, pose.y = float(world_pt[0]), float(world_pt[1])
                    self.pose_pub.publish(pose)
                    
                    # Dibujar robot para validar
                    cv2.circle(out, (int(world_pt[0]), int(world_pt[1])), 15, (0, 255, 0), -1)
                    cv2.putText(out, "ROBOT", (int(world_pt[0])+20, int(world_pt[1])), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                # Publicar imagen (siempre, para no ver negro en rviz)
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(out, "bgr8"))
        else:
            self.get_logger().info("Cámara activa pero no veo ningún ArUco...", once=True)

def main():
    rclpy.init()
    rclpy.spin(BoardProcessor())
    rclpy.shutdown()

if __name__ == '__main__':
    main()