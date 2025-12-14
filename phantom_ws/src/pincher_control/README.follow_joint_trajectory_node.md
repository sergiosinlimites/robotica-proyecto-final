## `follow_joint_trajectory_node.py`

Nodo de bajo nivel que implementa dos servidores de acción `FollowJointTrajectory` para conectar MoveIt 2 con los servos Dynamixel del Phantom X Pincher.

### Rol en el sistema

- Recibe trayectorias articulares desde los controladores de MoveIt (`joint_trajectory_controller`, `gripper_trajectory_controller`).
- Convierte posiciones en radianes/metros a ticks de los servos AX‑12A.
- Envía comandos de posición y velocidad a cada ID de servo a través de `dynamixel_sdk`.
- Publica el estado articular en `/joint_states` basándose en las posiciones **comandadas**.
- Permite controlar directamente el gripper con un simple `Bool` en `/set_gripper`.

### Entradas y salidas

- **Action servers** (`control_msgs/action/FollowJointTrajectory`):
  - `joint_trajectory_controller/follow_joint_trajectory` → brazo (4 articulaciones).
  - `gripper_trajectory_controller/follow_joint_trajectory` → gripper.
- **Suscripciones**:
  - `/set_gripper` (`std_msgs/Bool`): control directo del gripper (sin pasar por MoveIt).
- **Publicaciones**:
  - `/joint_states` (`sensor_msgs/JointState`): nombres y posiciones de todas las joints del brazo y del gripper.

### Mapeo de articulaciones a servos

En el constructor se define el diccionario `joint_to_id`:

```232:0:phantom_ws/src/pincher_control/pincher_control/follow_joint_trajectory_node.py
phantomx_pincher_arm_shoulder_pan_joint   → ID 1
phantomx_pincher_arm_shoulder_lift_joint  → ID 2
phantomx_pincher_arm_elbow_flex_joint     → ID 3
phantomx_pincher_arm_wrist_flex_joint     → ID 4
phantomx_pincher_gripper_finger1_joint    → ID 5
phantomx_pincher_gripper_finger2_joint    → ID 5  (mismo servo físico)
```

- `joint_sign` ajusta el sentido de giro (por ejemplo, hombro y codo tienen signo negativo).

### Ejecución de trayectorias

- `goal_callback`: valida que todas las joints del goal existen en `joint_to_id` y que hay al menos un `JointTrajectoryPoint`.
- `execute_callback`:
  1. Calcula un tiempo de inicio (`start_wall`).
  2. Recorre cada `JointTrajectoryPoint` de la trayectoria.
  3. Espera hasta `time_from_start` relativo a `start_wall`.
  4. Llama a `send_point(joint_names, point)` para enviar los comandos de posición.
  5. Al final marca el resultado como `SUCCESSFUL`.
- `send_point`: para cada joint del punto:
  - Actualiza `current_positions[j_name]` (para `/joint_states`).
  - Convierte la posición a ticks (`rad_to_dxl_tick` o `gripper_meter_to_tick`).
  - Envía `write2ByteTxRx` al servo correspondiente.
  - Evita enviar dos veces al mismo ID en el mismo punto (caso de las dos fingers del gripper).

### Control directo del gripper

- `set_gripper_callback` suscribe `/set_gripper` (`Bool`):
  - `True`  → abre el gripper (0 deg → 512 ticks, ~0.0158 m).
  - `False` → cierra el gripper (−80 deg → 239 ticks, ~0.001 m).
- Además de mover el servo real, actualiza las posiciones de `gripper_finger1_joint` y `gripper_finger2_joint` en `current_positions` para que `/joint_states` refleje el cambio.

### Parámetros

- `port` (`/dev/ttyUSB0` por defecto): puerto serie del bus Dynamixel.
- `baudrate` (por defecto 1000000).
- `joint_prefix` (`"phantomx_pincher_"`).
- `moving_speed`, `torque_limit`.
- `gripper_id` (ID del servo del gripper, por defecto 5).

### Uso típico

Este nodo se lanza desde `phantomx_pincher.launch.py` cuando `use_real_robot:=true`:

```bash
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py \
  use_real_robot:=true
```

MoveIt enviará automáticamente las trayectorias a los action servers correspondientes y el nodo se encargará de mover los servos físicos.
