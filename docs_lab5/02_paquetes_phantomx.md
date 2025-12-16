## Paquetes principales del proyecto PhantomX Pincher

### 1. `phantomx_pincher_description`

- **Rol**: Define la descripción del robot (URDF/Xacro), geometría, masas, colisiones y configuración base de `ros2_control`.
- **Contenido clave**:
  - `urdf/phantomx_pincher.urdf.xacro`: descripción completa del robot.
  - `urdf/phantomx_pincher_arm.xacro`: submodelo del brazo con límites articulares.
  - `phantomx_pincher.ros2_control`: definición de interfaces de `ros2_control` para `arm` y `gripper`.
  - `config/initial_joint_positions.yaml`: posiciones iniciales de articulaciones.
  - `launch/view.launch.py`: lanza `robot_state_publisher` + `rviz2` para visualizar el modelo.
- **Para qué lo usas en el lab**:
  - Para extraer longitudes y límites de las articulaciones (ayuda para DH).
  - Para visualizar el modelo en RViz.
  - Como base para que `robot_state_publisher` publique las TF.

---

### 2. `phantomx_pincher_moveit_config`

- **Rol**: Configuración de MoveIt 2 para el PhantomX Pincher.
- **Contenido clave**:
  - `config/*.yaml`: cinemática (`kinematics.yaml`), límites (`joint_limits.yaml`), planners (`ompl_planning.yaml`), controladores (`controllers_*.yaml`), etc.
  - `srdf/phantomx_pincher.srdf.xacro`: definición de grupos (`arm`, `gripper`), estados nombrados, etc.
  - `launch/move_group.launch.py`: lanza `robot_state_publisher`, `ros2_control_node`, `move_group` y RViz (según argumentos).
  - `rviz/moveit.rviz`: configuración predefinida de RViz para MoveIt.
- **Para qué lo usas**:
  - Planificar trayectorias en espacio articular y cartesiano.
  - Integrar con el nodo C++ `commander` y con `follow_joint_trajectory_node.py`.

---

### 3. `phantomx_pincher_commander_cpp`

- **Rol**: Implementa un “commander” en C++ que envuelve MoveIt y expone interfaces simples por tópicos.
- **Contenido clave**:
  - `src/commander_template.cpp`: nodo `commander` que usa `MoveGroupInterface` para:
    - Mover a named targets de MoveIt (`goToNamedTarget`).
    - Mover a un vector de juntas (`goToJointTarget`).
    - Mover a una pose cartesiana (`goToPoseTarget`).
  - Suscripciones:
    - `open_gripper` (`Bool`).
    - `joint_command` (`Float64MultiArray`).
    - `pose_command` (`phantomx_pincher_interfaces/PoseCommand`).
- **Uso en el lab**:
  - Ejemplo de cómo controlar el robot en espacio articular y cartesiano a través de tópicos sencillos.
  - Puedes inspirarte para la versión en Python + Toolbox.

---

### 4. `phantomx_pincher_interfaces`

- **Rol**: Define mensajes personalizados usados por el stack PhantomX.
- **Contenido clave**:
  - `msg/PoseCommand.msg`:
    - `float64 x, y, z`
    - `float64 roll, pitch, yaw`
    - `bool cartesian_path`
- **Uso**:
  - Lo consume el nodo `commander` para recibir comandos de pose cartesiana.

---

### 5. `pincher_control`

- **Rol**: Control directo de los servos Dynamixel + GUI + puente con MoveIt.
- **Contenido clave**:
  - `control_servo.py`:
    - Nodo `PincherController` (ROS 2) que habla con Dynamixel (`dynamixel_sdk`).
    - Publica `/joint_states` a partir de las posiciones de los motores.
    - Gestiona HOME, velocidad, torque, parada de emergencia.
    - GUI `PincherGUI` con 3 pestañas: sliders, valores manuales, RViz integrado.
  - `follow_joint_trajectory_node.py`:
    - Nodo `PincherFollowJointTrajectory` con dos ActionServer `FollowJointTrajectory`:
      - `/joint_trajectory_controller/follow_joint_trajectory` (brazo).
      - `/gripper_trajectory_controller/follow_joint_trajectory` (gripper).
    - Mapea nombres de joints a IDs de Dynamixel y ejecuta trayectorias.
  - `readme.md`: instrucciones rápidas de uso de `follow_joint_trajectory_node.py`.
- **Uso en el lab**:
  - Base para:
    - Mover el robot físico desde Python/ROS 2.
    - Implementar el movimiento entre poses HOME/objetivo por articulación.
    - Crear y extender la HMI pedida (pestañas, sliders, etc.).
  - Fuente de `/joint_states` para RViz y para obtener las configuraciones articulares en tiempo real.


