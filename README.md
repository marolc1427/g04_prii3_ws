# Sprint 5 — Eurobot 2026

![ROS2 Foxy](https://img.shields.io/badge/ROS2-Foxy-red?style=for-the-badge&logo=ros)
![Ubuntu 20.04](https://img.shields.io/badge/OS-Ubuntu_20.04-orange?style=for-the-badge&logo=ubuntu)
![OpenCV](https://img.shields.io/badge/OpenCV-4.6-green?style=for-the-badge&logo=opencv)
![Status](https://img.shields.io/badge/Status-Development-yellow?style=for-the-badge)

### ¿Cómo puedo ayudar?
* Proponer en el grupo reunirse para probar en el laboratorio los últimos cambios.

  
### Instrucciones y versiones:
* Se recomienda usar el workspace de ROS2 para usar los comandos explicados, sino, las rutas serán diferentes que las indicadas.
* Se debe de utilizar la versión de OpenCV 4.6, sino la detección de los ArUcos no será óptima para el trabajo.
* Se debe mantener el repositorio al día para que el resto del equipo pueda trabajar sin problemas.

## Simulación:

---

### 0) Clonar el repo (Terminal 1)

```bash
git clone https://github.com/marolc1427/g04_prii3_ws
```

---

### 1) Ejecutar el mundo en Gazebo (Terminal 1)
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

### 2) Lectura de ArUcos y visualización new_eurobot_basic.py (Terminal 2)

Este nodo extiende el antiguo `eurobot_basic.py` con publicaciones por ID y pequeños cambios en el formato.

**Diferencias clave**

- **Topics por ID**: publica de forma individual en:
	- `/overhead_camera/aruco_20`, `/overhead_camera/aruco_21`, `/overhead_camera/aruco_22`, `/overhead_camera/aruco_23`, `/overhead_camera/aruco_3`, `/overhead_camera/aruco_8`.
- **Contenido JSON por ID**: incluye `id`, `px`, `py`, `orientation` y, si hay estimación de pose, `rvec` y `tvec` del marcador.
- **Precisión**: valores redondeados a **4 decimales** en los topics por ID.
- **Imagen anotada**: igual que el nodo anterior, publica `/overhead_camera/image_annotated` para visualizar en RViz2.
- **Launch dedicado**: se lanza con `new_eurobot_basic.launch.py`.

**Cómo ejecutarlo**

```bash
source install/setup.bash
ros2 launch g04_prii3_eurobot_turtlebot new_eurobot_basic.launch.py
```

Opcional (visualización en RViz2): añade un display Image apuntando a `/overhead_camera/image_annotated`.

Comprobaciones por terminal:

```bash
ros2 topic echo /overhead_camera/aruco_20
ros2 topic echo /overhead_camera/aruco_8
```

Ejemplo de mensaje por ID:

```json
{"id":20,"px":961.824,"py":479.809,"orientation":-179.707,"rvec":[-0.0041,-3.0299,1.5708],"tvec":[0.1203,0.0301,0.8502]}
```

---

### 3) Movimiento del Robot (Terminal 3)

```bash
source install/setup.bash
ros2 run g04_prii3_eurobot_turtlebot new_aruco_go_to
```

---

## Ejecución en el tablero real

#### Recomendaciones:
* Usar un USB con el repositorio en vez de hacer un clone.
* Comprobar que el robot puede leer la información de los ArUcos mediante: 

```bash
ros2 topic list
```
Y también: 

```bash
ros2 topic echo /overhead_camera/aruco_20
ros2 topic echo /overhead_camera/aruco_8
```
---

## 1) Arranque de la cámara:

El código con la detección de los ArUcos ya está en el disco duro de la Jetson Nano. Por lo que solo debemos de hacer funcionar a la cámara y ejecutar el nodo de detección:

Terminal 1:
```bash
ros2 launch jetbot_pro_ros2 gscam.py
```

Terminal 2:
```bash
source install/setup.bash
ros2 launch g04_prii3_eurobot_turtlebot new_eurobot_basic.launch.py
```

Debemos de asegurarnos de estar conectados a la misma red con la cámara cenital y con el robot.

---

## 2) Arranque del robot:

Se recomienda usar la ssh (en todas las terminales que vamos a abrir) para no depender del cable HDMI para controlar el robot:

```bash
ssh -X jetbot@IP
```

El usuario y contraseña son:

```bash
Usuario: jetbot
Contraseña: jetbot
```

Terminal 1 (activación de los motores y LiDAR):

```bash
ros2 run jetbot_pro_ros2 jetbot
```

Terminal 2 (compilación y ejecución):
```bash
colcon build --packages-select g04_prii3_eurobot_turtlebot
source install/setup.bash
ros2 run g04_prii3_eurobot_turtlebot new_aruco_go_to
```
