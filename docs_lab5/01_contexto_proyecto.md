## Contexto actual del proyecto PhantomX Pincher (Lab 05)

Este workspace ya viene con casi todo el stack necesario para el PhantomX Pincher X100 en ROS 2:

- **`phantomx_pincher_description`**: modelo 3D, URDF/Xacro del robot, límites articulares y configuración básica de `ros2_control`.
- **`phantomx_pincher_moveit_config`**: configuración completa de MoveIt (grupos `arm` y `gripper`, planners, controladores, RViz).
- **`phantomx_pincher_commander_cpp`**: nodo C++ que usa MoveIt para mover el brazo en espacio articular y en espacio cartesiano.
- **`phantomx_pincher_interfaces`**: mensajes personalizados (por ejemplo `PoseCommand.msg`).
- **`pincher_control`**: nodos Python que hablan con los Dynamixel vía `dynamixel_sdk`, publican `/joint_states` e integran con MoveIt/RViz y una GUI.

---

## Modelo geométrico y relación con parámetros DH

En `phantomx_pincher_description/urdf/phantomx_pincher_arm.xacro` se define el brazo del PhantomX Pincher:

- Se declaran propiedades de las piezas (`ax12_height`, `f4_height`, etc.) que determinan las longitudes de eslabones.
- Se definen **límites articulares** en radianes para pan, shoulder, elbow y wrist:

- `pan_joint`: \([-150°, 150°]\)
- `shoulder_joint`: \([-120°, 120°]\)
- `elbow_joint`: \([-139°, 139°]\)
- `wrist_joint`: \([-98°, 103°]\)

Aunque el archivo no incluye explícitamente una tabla DH, sí contiene:

- La **estructura de eslabones y juntas** (`link`/`joint`) y
- Las transformaciones `origin xyz/rpy` entre eslabones.

A partir de estas transformaciones (o de medidas físicas) se pueden derivar los **parámetros DH** que te pide el laboratorio, y con ellos construir el modelo en el Robotics Toolbox u otra toolbox.

---

## Control de Dynamixel + ROS 2

### Nodo `PincherController` (`pincher_control/control_servo.py`)

Este script implementa un nodo ROS 2 que controla directamente los motores Dynamixel:

- Inicializa la comunicación serie usando `dynamixel_sdk` (`PortHandler`, `PacketHandler`).
- Configura:
  - **Puerto** (`/dev/ttyUSB1`),
  - **Baudrate** (1 Mbps),
  - **IDs de motores** (`[1,2,3,4,5]`),
  - **Torque enable**, límite de torque y velocidad,
  - **Posición inicial (HOME)** en ticks.

- Mantiene un vector `current_joint_positions` en radianes, con el siguiente mapeo a joints del URDF:

  - `ID 1` → `phantomx_pincher_arm_shoulder_pan_joint`
  - `ID 2` → `phantomx_pincher_arm_shoulder_lift_joint`
  - `ID 3` → `phantomx_pincher_arm_elbow_flex_joint`
  - `ID 4` → `phantomx_pincher_arm_wrist_flex_joint`
  - `ID 5` → `phantomx_pincher_gripper_finger1_joint`

- **Publica `sensor_msgs/JointState` en `/joint_states`** (10 Hz), de forma que:
  - `robot_state_publisher` y RViz puedan animar el modelo en tiempo real.

- Expone métodos internos para:
  - Mover un motor a una posición de ticks (`move_motor`).
  - Actualizar la velocidad de todos los motores (`update_speed`).
  - Enviar todos los motores a HOME (`home_all_motors`).
  - Ejecutar parada de emergencia (apaga torque) y reactivar torque.

Esto cumple parte de la guía del laboratorio en cuanto a:

- **Mover articulaciones individuales** (waist, shoulder, elbow, wrist, gripper).
- Publicar el estado articular para visualización.

---

### Nodo `PincherFollowJointTrajectory` (`pincher_control/follow_joint_trajectory_node.py`)

Este nodo implementa el puente estándar entre **MoveIt** y el **hardware real**:

- Crea dos **ActionServer `FollowJointTrajectory`**:

  - `/joint_trajectory_controller/follow_joint_trajectory` → grupo `arm` (4 DOF).
  - `/gripper_trajectory_controller/follow_joint_trajectory` → grupo `gripper`.

- Mapea nombres de joints (URDF) a IDs de Dynamixel:

  - `phantomx_pincher_arm_shoulder_pan_joint` → ID 1  
  - `phantomx_pincher_arm_shoulder_lift_joint` → ID 2  
  - `phantomx_pincher_arm_elbow_flex_joint` → ID 3  
  - `phantomx_pincher_arm_wrist_flex_joint` → ID 4  
  - `phantomx_pincher_gripper_finger1_joint` → ID 5  
  - `phantomx_pincher_gripper_finger2_joint` → ID 5 (mismo servo físico)

- Convierte posiciones articulares de MoveIt (radianes) a **ticks AX‑12** y manda los comandos usando `dynamixel_sdk`.
- Publica `/joint_states` con las posiciones **comandadas**, de forma que MoveIt y RViz conozcan el estado actual durante la ejecución de trayectorias.

Con esto, MoveIt puede:

- Planear trayectorias en espacio articular o cartesiano.
- Enviar goals `FollowJointTrajectory`.
- Hacer que el brazo físico ejecute la trayectoria.

---

## MoveIt + puente C++ (espacio articular y de la tarea)

### Paquete `phantomx_pincher_moveit_config`

Proporciona la configuración de MoveIt para el PhantomX:

- Grupos de joints (`arm`, `gripper`), límites articulares, parámetros de cinemática, OMPL, etc.
- `launch/move_group.launch.py` que levanta:
  - `robot_state_publisher`.
  - `ros2_control_node` con controladores fake (o reales si se implementa).
  - `move_group` (planner de MoveIt).
  - RViz opcionalmente.

### Nodo `commander` (`phantomx_pincher_commander_cpp/src/commander_template.cpp`)

- Crea dos `MoveGroupInterface`:
  - `arm` → control del brazo.
  - `gripper` → control del efector final.

- Se suscribe a:

  - `open_gripper` (`Bool`) → abrir/cerrar gripper con named targets (`gripper_open`, `gripper_closed`).
  - `joint_command` (`Float64MultiArray`) → vector de juntas para ir a una configuración articular.
  - `pose_command` (`PoseCommand`) → especifica `x, y, z, roll, pitch, yaw` + bandera `cartesian_path`.

- Implementa:
  - **`goToNamedTarget`**: ir a un named target de MoveIt (por ejemplo, `home` si lo defines en SRDF).
  - **`goToJointTarget`**: mover el brazo a un vector de 4–6 ángulos articulares.
  - **`goToPoseTarget`**: mover el TCP a una pose cartesiana con planificación por MoveIt, con opción de trayectorias cartesianas.

Con esto se cubre:

- **Control en espacio articular** mediante comandos de juntas.
- **Control en espacio cartesiano** (TCP) mediante `pose_command`.

---

## Interfaz gráfica / HMI actual (`control_servo.py`)

La clase `PincherGUI` implementa una GUI con `tkinter` y pestañas (`ttk.Notebook`):

### Pestaña 1 – Control por Sliders (espacio articular en “ticks”)

- Un slider por motor (IDs 1–5) con rango `0–4095`.
- Un slider de velocidad global.
- Cada vez que mueves un slider, si la velocidad > 0 y no hay emergencia:
  - Llama a `controller.move_motor` para mandar la posición al Dynamixel.
  - Actualiza la posición mostrada y el estado de la GUI.

Esta pestaña ya realiza **control articular con deslizadores**, aunque en unidades de ticks, no en grados, y sin verificar exactamente los límites mecánicos individuales (usa el rango de los servos).

### Pestaña 2 – Control por Valores Manuales

- Entradas numéricas (`Entry`) para cada motor, con texto “Valor (0-4095)”.
- Botón “Mover Motor” para cada articulación.
- Botón “MOVER TODOS LOS MOTORES” que lee todos los entries y envía comandos.
- Validación básica:
  - Verifica que el valor es un entero.
  - Verifica que está en `0–4095`.
  - Considera velocidad = 0 y estado de emergencia.

Esta pestaña implementa una forma de **ingreso numérico articular**, aunque de nuevo en ticks.  
Para el laboratorio, podrías extenderla para que el usuario ingrese grados y se conviertan internamente a ticks, respetando los límites de cada joint.

### Pestaña 3 – Visualización RViz

- Botón **“LANZAR RViz”**:
  - Ejecuta `ros2 launch phantomx_pincher_description view.launch.py`, que levanta:
    - `robot_state_publisher` con la descripción del robot.
    - `rviz2` con una configuración específica (`view.rviz`).
- Botón **“DETENER RViz”** para terminar el proceso.
- Muestra etiquetas con las **posiciones articulares en radianes** de cada joint, leyendo `controller.current_joint_positions`.
- Actualiza la GUI periódicamente para reflejar la posición real/comandada.

Con esto se logra:

- **Visualizar el modelo 3D en RViz** sincronizado con el robot físico.
- Ver numéricamente los ángulos de cada articulación (en radianes).

---

## Qué falta frente a los requisitos del laboratorio

Lo que ya está:

- Modelo geométrico completo del robot en URDF/Xacro.
- Límites articulares codificados en el Xacro.
- Comunicación con Dynamixel vía `dynamixel_sdk`.
- Publicación de `/joint_states` y visualización en RViz.
- Puente con MoveIt (`FollowJointTrajectory` + `commander`).
- GUI con:
  - Sliders para movimiento articular.
  - Entradas numéricas por motor.
  - Lanzador de RViz y visualización de estados.

Lo que falta implementar explícitamente:

- **Tabla DH** explícita y diagrama del robot con esos parámetros.
- Script ROS 2 en Python que:
  - Realice el movimiento secuencial entre HOME y una pose objetivo por articulación (base → shoulder → elbow → wrist) con pequeñas esperas.
- Scripts ROS 2 (en Python) para:
  - Publicar en tópicos de control articular con límites en **grados** validados.
  - Suscribirse a los controladores de articulación o a `/joint_states` y devolver/imprimir los 5 ángulos en grados respecto a HOME.
- Script de **Python + ROS + Toolbox** que:
  - Envíe una configuración articular (por ejemplo las 5 poses de la guía) al robot.
  - Calcule la cinemática directa con el modelo DH.
  - Grafique la pose del robot y la compare con RViz / robot real.
- Extender la GUI para:
  - Incluir datos del grupo (nombres, logos).
  - Botones para seleccionar y enviar las 5 poses propuestas.
  - Pestaña de **control en espacio de la tarea** (sliders X, Y, Z, R, P, Y) conectada a `pose_command`.
  - Pestaña de **visualización numérica de la pose cartesiana (X, Y, Z, RPY)** obtenida a partir de la cinemática directa o de MoveIt.


