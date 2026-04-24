# Sprint 8 - Eurobot 2026

---

## Arquitectura y metodología de trabajo:

En este apartado se explica la metodología de trabajo para el sprint 8. Por lo que se recomienda leer este apartado y comprenderlo en profunidad. Ante cualquier duda / idea / problema, preguntad en el grupo de Whatsapp o en clase.

> [!IMPORTANT]
> El Sprint 8 aún se encuentra en desarrollo, por lo que es posible que haya cambios en la metodología de trabajo o en las instrucciones.

---

## Ordenador Fijo, Ubuntu 20.04 LTS y ROS2 Foxy

![Ubuntu 20.04](https://img.shields.io/badge/OS-Ubuntu_20.04-orange?style=for-the-badge&logo=ubuntu)
![ROS2 Foxy](https://img.shields.io/badge/ROS2-Foxy-red?style=for-the-badge&logo=ros)
![OpenCV](https://img.shields.io/badge/OpenCV-4.6-green?style=for-the-badge&logo=opencv)

El ordenador fijo se ejecutan los nodos:
* Nodo para arrancar la cámara cenital.
* Nodo para la homografía.
* Nodo para la detección de los ArUcos.

El nodo de la cámara cenital se encarga de a través del puerto USB publicar la imagen de la webcam por un topic. El nodo de homografía se encarga de transformar los píxeles en coordenadas (x,y). Posteriormente, el nodo de detección de ArUcos se subscribe al topic del tablero con homografía, para publicar en topics en fomrato JSON las posiciones en coordenadas, las posiciones de los ArUcos del tablero (20,21,22,23) y del robot móvil (3). 

Para cada ArUco, se publica un topic (/detection/aruco_{id_}) con la información de su posición (px, py), orientación e ID. Con el fin de que el robot, con un ArUco encima, sepa dirijirse a las posiciones deseadas. 

Se recomienda usar el siguiente comando para observar los topics publicados:
```bash
ros2 topic echo /topic_name
```

Se recomienda usar el siguiente comando para observar la detección de los ArUcos en RViz2:
```bash
rviz2
```

### Ejecución:

1. Terminal 1, compilar y lanzar el nodo de arranque de la cámara cenital:

```bash
colcon build --packages-select sprint_8
source install/setup.bash
ros2 run sprint_8 webcam_node
```

2. Terminal 2, lanzar el nodo de la homografía:

```bash
ros2 run sprint_8 homography_node
```

3. Terminal 3, lanzar el nodo de la detección de ArUcos:

```bash
ros2 run sprint_8 aruco_detector_node
```

---

## Robot móvil, Ubuntu 22.04 LTS y ROS2 Humble

![Ubuntu 22.04](https://img.shields.io/badge/OS-Ubuntu_22.04-orange?style=for-the-badge&logo=ubuntu) 
![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-red?style=for-the-badge&logo=ros)
![OpenCV](https://img.shields.io/badge/OpenCV-4.6-green?style=for-the-badge&logo=opencv)

En el robot móvil se ejecuta lo siguiente:
* Fichero .ino con el mapeo de los pines de los motores y la ventosa.
* Nodo con el puerto USB a la Arduino que publica un topic para el uso de la ventosa (/ventosa_cmd) y para los motores (/cmd_vel)
* Nodo de movimiento del robot y recogida de piezas. 

En cuanto al nodo de movimiento, se ha implementado una FSM donde cada estado representa una fase del proceso de movimiento y recogida de piezas. 

La FSM tiene los siguientes estados:
1. **Aproximación a pieza**: el robot móvil se aproxima a la pieza colocada SIEMPRE en la misma posición para recogerla (posición hardcodeada en coordenadas). Pasa al estado de "Recogida de pieza".
2. **Recogida de pieza**: el robot activa el mecanismo de recogida para coger la pieza. Para ello, el robot se encara a la pieza y mueve el brazo para recogerla. Activa las ventosas y levanta la pieza del tablero. Pasa al estado de "Dejada de pieza".
3. **Dejada de pieza**: Una vez recogida la pieza, el robot se dirige hacia la zona de dejada, hardcodeada también. Si el robot se acerca lo suficiente a la zona de dejada, suelta la pieza y finaliza la ejecución.

## Ejecución:

1. Terminal 1, lanzar los topics del brazo:

```bash
cd ~/brazo_ws 
source install/setup.bash 
ros2 launch brazo_pkg bringup.launch.py 
```

2. Terminal 2, lanzar el topic de la cámara:

```bash
ros2 launch astra_camera astra_pro_plus.launch.xml 
```

3. Terminal 3, lanzar los topics de la ventosa y los motores:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run robot_control robot_node
```

4. Terminal 4, ejecución de nodo principal:

```bash
source install/setup.bash
ros2 run sprint_8 pick_and_place
```