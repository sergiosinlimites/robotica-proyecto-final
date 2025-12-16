## `phantomx_pincher.launch.py`

Lanzador principal del **stack de movimiento** del Phantom X Pincher con MoveIt 2, tanto en simulación como en el robot real.

### Rol en el sistema

- Crea el árbol de TF (`robot_state_publisher`) a partir del URDF (`phantomx_pincher.urdf.xacro`).
- Arranca **MoveIt 2** (`move_group.launch.py`) en modo simulación o en modo real.
- En modo real, lanza `pincher_control/follow_joint_trajectory` para conectar MoveIt con los motores Dynamixel.
- Lanza el nodo `phantomx_pincher_commander_cpp/commander`, que escucha `/pose_command` y `joint_command` y genera trayectorias de MoveIt.
- Opcionalmente lanza `pincher_control/clasificador_node` (máquina de estados de pick&place).

### Argumentos principales

- `use_real_robot` (`true` / `false`):
  - `false` (por defecto): usa `controller_manager` + `ros2_control_node` con controladores fake (`joint_trajectory_controller`, `gripper_trajectory_controller`) y lanza MoveIt en modo simulación.
  - `true`: **no** crea `ros2_control_node`; en su lugar lanza `pincher_control/follow_joint_trajectory` y arranca MoveIt en modo `ros2_control_plugin:=real`.
- `start_clasificador` (`true` / `false`):
  - Si es `true`, lanza el nodo `pincher_control/clasificador_node` que implementa la FSM de pick&place.

### Nodos que lanza

- `robot_state_publisher`: publica el URDF con el parámetro `robot_description` generado por `xacro`.
- `phantomx_pincher_commander_cpp/commander`:
  - Parametrizado con `robot_description`, `robot_description_semantic` (SRDF) y `kinematics.yaml`.
  - Suscribe:
    - `/pose_command` → llama a `goToPoseTarget()` en MoveIt.
    - `/joint_command` → llama a `goToJointTarget()` para mover directamente las 4 articulaciones.
    - `/open_gripper` (`Bool`) → abre/cierra el gripper vía MoveIt.
- **Modo SIM (`use_real_robot:=false`)**:
  - `controller_manager/ros2_control_node` con `controllers_position.yaml`.
  - Spawners para `joint_state_broadcaster`, `joint_trajectory_controller`, `gripper_trajectory_controller`.
  - `phantomx_pincher_moveit_config/launch/move_group.launch.py` con:
    - `ros2_control=""` (no crea otro ros2_control).
    - `manage_controllers:=false`.
    - `enable_rviz:=true`.
- **Modo REAL (`use_real_robot:=true`)**:
  - `pincher_control/follow_joint_trajectory` (control de Dynamixel real).
  - `phantomx_pincher_moveit_config/launch/move_group.launch.py` con:
    - `ros2_control:=false`.
    - `ros2_control_plugin:=real`.
    - `manage_controllers:=false`.
- `pincher_control/clasificador_node` (si `start_clasificador:=true`):
  - FSM de pick&place que escucha `/figure_type` (salida de YOLO) y publica `/pose_command`, `/set_gripper`, `/routine_busy` y `joint_command`.

### Uso típico

- **Simulación**:

```bash
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py \
  use_real_robot:=false \
  start_clasificador:=true
```

- **Robot real**:

```bash
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py \
  use_real_robot:=true \
  start_clasificador:=true
```
