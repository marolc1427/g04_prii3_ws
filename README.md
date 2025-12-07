# Sprint 4 — Turtlebot en la Eurobot 2026

![ROS2 Foxy](https://img.shields.io/badge/ROS2-Foxy-red?style=for-the-badge&logo=ros)
![Ubuntu 20.04](https://img.shields.io/badge/OS-Ubuntu_20.04-orange?style=for-the-badge&logo=ubuntu)
![OpenCV](https://img.shields.io/badge/OpenCV-4.2-green?style=for-the-badge&logo=opencv)
![Status](https://img.shields.io/badge/Status-Development-yellow?style=for-the-badge)

### Actualidad del proyecto:
* Lectura de ArUcos (FOV, archi.py, movimiento del robot...) y clarificar la versión de OpenCV
* Movimiento del robot sabiendo los ArUcos del paso anterior
* Podríamos crear un archivo .sh para ejecutar todo el proyecto desde una única terminal 
* Falta actualizar el readme con las novedades anteriores ya implementadas

---

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

## 2) Ejecución del launch lectura de ArUcos y movimiento del Waffle (Terminal 2)

Actualmente, NO funcional

```bash
source install/setup.bash
ros2 launch g04_prii3_eurobot_turtlebot eurobot_basic_launch.py
```