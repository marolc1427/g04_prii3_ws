# Sprint 8 - Eurobot 2026

---

## Arquitectura y metodología de trabajo:

En este apartado se explica la metodología de trabajo para el sprint 8. Por lo que se recomienda leer este apartado y comprenderlo en profunidad. Ante cualquier duda / idea / problema, preguntad en el grupo de Whatsapp o en clase.

> [!IMPORTANT]
> El Sprint 8 aún se encuentra en desarrollo, por lo que es posible que haya cambios en la metodología de trabajo o en las instrucciones.

---

### Ordenador Fijo, Ubuntu 20.04 LTS y ROS2 Foxy

![Ubuntu 20.04](https://img.shields.io/badge/OS-Ubuntu_20.04-orange?style=for-the-badge&logo=ubuntu)
![ROS2 Foxy](https://img.shields.io/badge/ROS2-Foxy-red?style=for-the-badge&logo=ros)
![OpenCV](https://img.shields.io/badge/OpenCV-4.6-green?style=for-the-badge&logo=opencv)

El ordenador fijo funciona como centro de operaciones, en él se ejecutan los nodos:
* Nodo para arrancar la cámara cenital.
* Nodo para la homografía (si se desea).
* Nodo para la detección de los ArUcos.

El único nodo "que es leído" por el robot móvil es el de detección de ArUcos. Este nodo publica un topic por cada ID de ArUco detectado que no sea 20, 21, 22 ni 23. Estos IDs son de los ArUcos del tablero, por lo que no son piezas a recoger. 

Para cada ArUco, se publica un topic con la información de su posición (px, py) y orientación. Con el fin de que el robot, con un ArUco encima y el ArUco de la pieza, sepa dirijirse hacia ella. 

Se recomienda usar el siguiente comando para observar los topics publicados:
```bash
ros2 topic echo /topic_name
```

Se recomienda usar el siguiente comando para observar la detección de los ArUcos en RViz2:
```bash
rviz2
```

---

### Robot móvil, Ubuntu 22.04 LTS y ROS2 Humble

![Ubuntu 22.04](https://img.shields.io/badge/OS-Ubuntu_22.04-orange?style=for-the-badge&logo=ubuntu) 
![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-red?style=for-the-badge&logo=ros)
![OpenCV](https://img.shields.io/badge/OpenCV-4.6-green?style=for-the-badge&logo=opencv)

En el robot móvil se ejecuta lo siguiente:
* Nodo de movimiento del robot y recogida de piezas. 
* Comunicación con la placa Arduino para el control de los motores.

En cuanto al nodo de movimiento, se ha implementado una FSM donde cada estado representa una fase del proceso de movimiento y recogida de piezas. 

La FSM tiene los siguientes estados:
1. **Aproximación a pieza**: el robot móvil se aproxima a la pieza a recoger con el identificador ArUco más bajo utilizando la información de posición y orientación del ArUco. Si el robot se acerca lo suficiente a la pieza, pasa al estado de "Recogida de pieza".
2. **Recogida de pieza**: el robot activa el mecanismo de recogida para coger la pieza. Para ello, el robot se encara a la pieza y mueve el brazo para recogerla. Activa las ventosas y levanta la pieza del tablero. Si la recogida es exitosa, pasa al estado de "Dejada de pieza".
3. **Dejada de pieza**: Una vez recogida la pieza, el robot se dirige hacia la zona de dejada. Si el robot se acerca lo suficiente a la zona de dejada, suelta la pieza y vuelve al estado de "Aproximación a pieza".

---

## Ejecución:

---

### Ordenador fijo:

1. Terminal 1, compilar y lanzar el nodo de arranque de la cámara cenital:

```bash
colcon build --packages-select sprint_8
source install/setup.bash
ros2 run sprint_8 webcam_node
```

2. Terminal 2, lanzar el nodo de la homografía:

```bash
ros2 run sprint_8 aruco_detector_node
```

3. Terminal 3, lanzar el nodo de movimiento:

```bash
ros2 run sprint_8 movimiento
```

---

### Robot móvil:

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

