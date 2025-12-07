# Sprint 4 — Turtlebot en la Eurobot 2026

![ROS2 Foxy](https://img.shields.io/badge/ROS2-Foxy-red?style=for-the-badge&logo=ros)
![Ubuntu 20.04](https://img.shields.io/badge/OS-Ubuntu_20.04-orange?style=for-the-badge&logo=ubuntu)
![OpenCV](https://img.shields.io/badge/OpenCV-4.2-green?style=for-the-badge&logo=opencv)
![Status](https://img.shields.io/badge/Status-Development-yellow?style=for-the-badge)

### ¿Cómo puedo ayudar?
* **Correción de la Detección de ArUcos**: COMPLETADO. Implementado el reconocimiento de ArUcos del tablero con OpenCV 4.6 (compatible 4.2) dentro del nodo [`eurobot_basic.py`](./src/g04_prii3_eurobot_turtlebot/g04_prii3_eurobot_turtlebot/eurobot_basic.py). Ahora se publica:
	- `'/overhead_camera/aruco_detections'` (`std_msgs/String`) con detecciones en JSON compacto.
	- `'/overhead_camera/image_annotated'` (`sensor_msgs/Image`) para visualización en RViz2. Este tópico es temporal y podría eliminarse en el futuro.
* Movimiento del robot sabiendo los ArUcos del paso anterior. Se debe de crear un nuevo nodo al lado de  [`eurobot_basic.py`](./src/g04_prii3_eurobot_turtlebot/g04_prii3_eurobot_turtlebot/eurobot_basic.py) e incluirlo en el launch y en el  [`setup.py`](./src/g04_prii3_eurobot_turtlebot/setup.py)
* Podríamos crear un archivo .sh para ejecutar todo el proyecto desde una única terminal.
* Falta actualizar el readme con las novedades anteriores ya implementadas.
  
### Instrucciones y versiones:
* Se recomienda usar el workspace de ROS2 para usar los comandos explicados, sino, las rutas serán diferentes que las indicadas.

---

## 0) Clonar el repo (Terminal 1)

```bash
git clone https://github.com/marolc1427/g04_prii3_ws
```

---

## 1) Ejecutar el mundo en Gazebo (Terminal 1)
Primeramente, se debe de extraer en una ruta conocida para Gazebo los modelos que vamos a utilizar (ArUcos, tablero, valla y robot waffle personalizado).

```bash
unzip src/g04_prii3_eurobot_turtlebot/worlds/models/modelos.zip -d ~/.gazebo/models/
```

Tras ello, ya se puede compilar y cargar en nuestra terminal el entorno de ROS2.

```bash
colcon build --packages-select g04_prii3_eurobot_turtlebot
export TURTLEBOT3_MODEL=waffle
source install/setup.bash
ros2 launch g04_prii3_eurobot_turtlebot eurobot_world.launch.py
```

---

## 2) Lectura de ArUcos y visualización (Terminal 2)

Funcional: detección de ArUcos y publicación de tópicos.

```bash
source install/setup.bash
# Nodo de detección (también accesible vía launch)
ros2 run g04_prii3_eurobot_turtlebot eurobot_basic
```

Opcional (vía launch):
```bash
source install/setup.bash
ros2 launch g04_prii3_eurobot_turtlebot eurobot_basic_launch.py
```

Visualización en RViz2 (añade un display Image apuntando a `/overhead_camera/image_annotated`):
```bash
rviz2
```

Comprobación por terminal de las detecciones:
```bash
ros2 topic echo /overhead_camera/aruco_detections
```
