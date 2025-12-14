## `phantomx_pincher_commander_cpp` – `commander` (C++)

Nodo C++ que actúa como puente entre los comandos de alto nivel (`/pose_command`, `joint_command`, `/open_gripper`) y las capacidades de planificación/ejecución de **MoveIt 2**.

### Rol en el sistema

- Suscribe a **órdenes de movimiento** generadas por nodos de control (por ejemplo, `clasificador_node`, `routine_manager`).
- Traduce esas órdenes en llamadas a `MoveGroupInterface` (`setApproximateJointValueTarget`, `plan`, `execute`).
- Es el único que habla directamente con MoveIt (el resto de nodos publican en `/pose_command` y `/joint_command`).

### Entradas y salidas

- **Suscripciones**:
  - `pose_command` (`phantomx_pincher_interfaces/ PoseCommand`):
    - `x, y, z, roll, pitch, yaw` en el frame base (`phantomx_pincher_base_link`).
    - `cartesian_path` (`bool`): si `true`, usa `computeCartesianPath`; si `false`, usa IK + planificación en espacio articular.
  - `joint_command` (`example_interfaces/Float64MultiArray`): vector `[q1, q2, q3, q4, ...]` en radianes para mover directamente las articulaciones del brazo.
  - `open_gripper` (`example_interfaces/Bool`): `True` → abre el gripper, `False` → cierra.
- **Salidas**:
  - No publica nuevos tópicos; su efecto se ve como movimiento del robot y en la actualización de `joint_states` (vía `follow_joint_trajectory`).

### Lógica de planificación (`goToPoseTarget`)

```100:125:phantom_ws/src/phantomx_pincher_commander_cpp/src/commander_template.cpp
// Construir PoseStamped en el frame de planificación de MoveIt
arm_->setStartStateToCurrentState();
if (!cartesian_path) {
    arm_->setGoalPositionTolerance(0.001);
    arm_->setGoalOrientationTolerance(3.14159); // permitir cualquier orientación

    if (arm_->setApproximateJointValueTarget(target_pose, "")) {
        planAndExecute(arm_);
    } else {
        RCLCPP_WARN(...);
        if (is_home_pose && home_joint_fallback_) {
            // Fallback: mover directamente a [0,0,0,0]
            std::vector<double> home_joints = {0.0, 0.0, 0.0, 0.0};
            goToJointTarget(home_joints);
        }
    }
}
```

- `is_home_pose` se evalúa como `|x| < 0.05`, `|y| < 0.05`, `z > 0.35`.
- `setApproximateJointValueTarget` permite a MoveIt buscar soluciones de IK aproximadas (mejor para robots de 4 GDL que no pueden cumplir orientaciones exactas).
- Se ha eliminado la restricción blanda sobre el codo (`JointConstraint`) para evitar errores del tipo *"Attempted to merge incompatible constraints for joint '...elbow_flex_joint'"* y mejorar la tasa de éxito del planificador OMPL.

### Lógica de `joint_command`

```199:207:phantom_ws/src/pincher_control/pincher_control/follow_joint_trajectory_node.py
void jointCmdCallback(const FloatArray &msg)
{
    auto joints = msg.data;
    if (joints.size() >= 4) {
        std::vector<double> arm_joints = {joints[0], joints[1], joints[2], joints[3]};
        goToJointTarget(arm_joints);
    }
}
```

- Permite enviar un vector simple de 4 ángulos (en radianes) para posicionar el brazo sin usar IK.
- `clasificador_node` lo usa al final de la rutina para llevar el robot a HOME real (`[0,0,0,0]`).

### Uso típico

El nodo se lanza automáticamente desde `phantomx_pincher.launch.py` tanto en sim como en real. Los usuarios normalmente **no lo ejecutan directamente**, sino que publican en `/pose_command` o `joint_command` desde sus propios nodos.
