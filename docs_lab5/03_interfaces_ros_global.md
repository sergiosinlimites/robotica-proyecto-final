## Nodos, tópicos, acciones y servicios relevantes del proyecto

> Nota: aquí se resumen las interfaces ROS 2 más importantes.  
> Los detalles exactos (por ejemplo todos los tópicos internos de MoveIt) se pueden ver con `ros2 node list`, `ros2 topic list`, etc., cuando el sistema está corriendo.

---

## 1. Paquete `pincher_control`

### 1.1 Nodo `pincher_controller` (`control_servo.py`)

- **Tipo**: `rclpy.node.Node`
- **Rol**: Hablar directamente con los Dynamixel y publicar el estado articular del robot.

**Parámetros** (ejemplos por defecto):

- `port` (`string`): `/dev/ttyUSB1`
- `baudrate` (`int`): `1000000`
- `dxl_ids` (`int[]`): `[1, 2, 3, 4, 5]`
- `goal_positions` (`int[]`): posición inicial de cada motor en ticks.
- `moving_speed` (`int`): velocidad global.
- `torque_limit` (`int`): límite de torque.

**Tópicos:**

- Publica:
  - `/joint_states` (`sensor_msgs/JointState`):
    - **`name[]`**:
      - `phantomx_pincher_arm_shoulder_pan_joint`  
      - `phantomx_pincher_arm_shoulder_lift_joint`  
      - `phantomx_pincher_arm_elbow_flex_joint`  
      - `phantomx_pincher_arm_wrist_flex_joint`  
      - `phantomx_pincher_gripper_finger1_joint`
    - **`position[]`**: posiciones en radianes calculadas a partir de los ticks.

- Suscribe:
  - No suscribe a tópicos externos para control; los comandos se originan desde la GUI interna.

**Servicios / acciones:**

- No expone servicios ni acciones ROS 2 explícitos.  
  El control es interno vía métodos (por ejemplo `move_motor`, `home_all_motors`, `emergency_stop`), llamados desde la GUI.

---

### 1.2 Nodo `pincher_follow_joint_trajectory` (`follow_joint_trajectory_node.py`)

- **Tipo**: `rclpy.node.Node`
- **Rol**: Puente entre MoveIt (trayectorias en radianes) y los motores Dynamixel (ticks).

**Parámetros principales:**

- `port`: `/dev/ttyUSB1`
- `baudrate`: `1000000`
- `joint_prefix`: `"phantomx_pincher_"`
- `moving_speed`: velocidad de los servos.
- `torque_limit`: límite de torque.
- `gripper_id`: ID del servo que mueve el gripper (típicamente 5).

**Acciones (`control_msgs/action/FollowJointTrajectory`):**

- Servidores de acción:

  - `joint_trajectory_controller/follow_joint_trajectory`  
    - Grupo objetivo: **`arm`** (4 DOF).
    - `goal.trajectory.joint_names` típicos:
      - `phantomx_pincher_arm_shoulder_pan_joint`
      - `phantomx_pincher_arm_shoulder_lift_joint`
      - `phantomx_pincher_arm_elbow_flex_joint`
      - `phantomx_pincher_arm_wrist_flex_joint`

  - `gripper_trajectory_controller/follow_joint_trajectory`  
    - Grupo objetivo: **`gripper`**.
    - `goal.trajectory.joint_names` típicos:
      - `phantomx_pincher_gripper_finger1_joint`
      - `phantomx_pincher_gripper_finger2_joint`
    - Internamente ambos se mapean al mismo ID (`gripper_id`).

**Tópicos:**

- Publica:
  - `joint_states` (`sensor_msgs/JointState`):
    - Nombres: todas las joints controladas (brazo + gripper + `gripper_joint` pasivo).
    - Posiciones: valores COMANDADOS en radianes (no necesariamente la lectura física).

- Suscribe:
  - Los tópicos internos del sistema de acciones:
    - `joint_trajectory_controller/follow_joint_trajectory/_action/*`
    - `gripper_trajectory_controller/follow_joint_trajectory/_action/*`
  - Normalmente no hace falta tratarlos uno por uno; se usan al enviar goals con `ros2 action send_goal` o desde MoveIt.

**Uso típico:**

- Terminal 1:

  ```bash
  ros2 run pincher_control follow_joint_trajectory
  ```

- Terminal 2:
  - Enviar un goal a la acción del brazo con:

  ```bash
  ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory "{ ... }"
  ```

  - O dejar que MoveIt lo use automáticamente cuando ejecutas un plan.

---

## 2. Paquete `phantomx_pincher_commander_cpp`

### Nodo `commander` (`commander_template.cpp`)

- **Tipo**: `rclcpp::Node`
- **Rol**: Ofrecer una interfaz sencilla basada en tópicos para controlar el brazo y el gripper usando MoveIt.

**Subscripciones:**

- `open_gripper` (`example_interfaces/Bool`):
  - `true` → manda al named target `gripper_open`.
  - `false` → manda al named target `gripper_closed`.

- `joint_command` (`example_interfaces/Float64MultiArray`):
  - Llama a `goToJointTarget(joints)` en el grupo `arm`.
  - Espera típicamente un vector de 4–6 valores (según configuración), en radianes.

- `pose_command` (`phantomx_pincher_interfaces/PoseCommand`):
  - Campos:
    - `x, y, z` (m).
    - `roll, pitch, yaw` (rad).
    - `cartesian_path` (`bool`).
  - Llama a `goToPoseTarget`, que:
    - Si `cartesian_path == false`: planifica en espacio de configuración.
    - Si `cartesian_path == true`: planifica una trayectoria cartesiana (waypoints).

**Tópicos publicados:**

- No publica directamente estados; delega en MoveIt, `robot_state_publisher` y los controladores.  
  La ejecución real de las trayectorias se encamina a través de los controladores de MoveIt (`FollowJointTrajectory`, etc.).

---

## 3. Paquete `phantomx_pincher_description`

### Nodo `robot_state_publisher` (desde `launch/view.launch.py`)

Lanzado por:

```bash
ros2 launch phantomx_pincher_description view.launch.py
```

**Nodos:**

- `robot_state_publisher`:
  - Publica transformaciones TF a partir de:
    - `robot_description` (URDF generado desde Xacro).
    - `joint_states` (producido por otros nodos como `PincherController`).

- `rviz2`:
  - Visualiza el modelo del robot y/o la planificación.

**Tópicos relevantes:**

- Suscribe:
  - `/joint_states` (`sensor_msgs/JointState`) → para saber las posiciones de las articulaciones.

- Publica:
  - `/tf` y `/tf_static` → árbol de transformaciones del robot.

---

## 4. Paquete `phantomx_pincher_moveit_config`

Al lanzar, por ejemplo:

```bash
ros2 launch phantomx_pincher_moveit_config move_group.launch.py \
  ros2_control:=false \
  ros2_control_plugin:=real \
  manage_controllers:=false
```

se arrancan típicamente:

**Nodos:**

- `robot_state_publisher` (si no lo arrancas por fuera).
- `controller_manager` / `ros2_control_node` (para controladores fake o reales, según config).
- `move_group` (nodo principal de MoveIt).
- `rviz2` (opcional, según argumentos `enable_rviz`, `rviz_config`).

**Interfaces relevantes:**

- **Acciones:**
  - `move_group` → múltiples acciones de MoveIt (`MoveGroup`, `ExecuteTrajectory`, etc.).
  - Controladores:
    - `/joint_trajectory_controller/follow_joint_trajectory` (si se configuran controladores internos de `ros2_control`).
    - `/gripper_trajectory_controller/follow_joint_trajectory`.

- **Tópicos:**
  - `/joint_states` (MoveIt lo usa, pero típicamente lo publican otros nodos).
  - `/move_group/status`, `/move_group/goal`, `/move_group/result`, etc.  
    (para interacción avanzada con MoveIt, normalmente gestionado por `MoveGroupInterface` en C++ o Python).

---

## 5. Resumen para el laboratorio

- **Para mover el robot en espacio articular**:
  - Usar `pincher_control/control_servo.py` (sliders, entradas numéricas).
  - O publicar en `joint_command` (si usas el nodo `commander`).
  - O enviar trayectorias a las acciones `FollowJointTrajectory`.

- **Para mover el robot en espacio de la tarea (TCP)**:
  - Usar el tópico `pose_command` del nodo `commander`.
  - O usar directamente MoveIt (RViz MotionPlanning, scripts propios).

- **Para visualizar el modelo y el estado**:
  - Lanzar `phantomx_pincher_description/view.launch.py` o `phantomx_pincher_moveit_config/move_group.launch.py`.
  - Asegurarte de que alguien publica `/joint_states` (`PincherController` o los controladores de `ros2_control`).

- **Para integrar con Toolbox y DH**:
  - Usar los parámetros DH derivados del URDF + los valores articulares (de `/joint_states` o del nodo que crees) para:
    - Calcular la pose del TCP.
    - Graficar el robot y compararlo con RViz / robot real.


