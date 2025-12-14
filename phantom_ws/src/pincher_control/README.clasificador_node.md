## `clasificador_node.py`

Nodo de alto nivel que implementa la **máquina de estados de pick & place** para el Phantom X Pincher.

### Rol en el sistema

- Recibe el tipo de figura detectada por YOLO desde `/figure_type` (por ejemplo, `cubo`).
- Mapea cada figura a una caneca (`caneca_roja`, `caneca_verde`, `caneca_azul`, `caneca_amarilla`).
- Ejecuta una secuencia de movimientos (HOME → zona de recolección → caneca → HOME) usando MoveIt a través de `commander`.
- Controla el gripper directamente mediante `/set_gripper`.
- Publica `/routine_busy` para pausar el reconocimiento mientras dura la rutina.

### Entradas y salidas

- **Suscripciones**:
  - `/figure_type` (`std_msgs/String`): tipos de figura (`cubo`, `cilindro`, `pentagono`, `hexagono`, `rectangulo`).
- **Publicaciones**:
  - `/pose_command` (`phantomx_pincher_interfaces/PoseCommand`): posiciones cartesianas (x, y, z, roll, pitch, yaw, `cartesian_path`).
  - `/set_gripper` (`std_msgs/Bool`): `True` abre, `False` cierra el gripper.
  - `/routine_busy` (`std_msgs/Bool`): `True` mientras la FSM está ejecutando una rutina; `False` cuando termina o se aborta.
  - `joint_command` (`example_interfaces/Float64MultiArray`): al final de la rutina envía `[0.0, 0.0, 0.0, 0.0]` para poner todas las articulaciones del brazo en HOME real.

### Estados principales de la FSM

- `IDLE`: esperando una nueva figura en `/figure_type`.
- `MOVING_TO_HOME_START`: ir a la pose `home` (posición segura inicial).
- `MOVING_TO_SAFE_POS_1` (`recoleccion_1`): aproximación sobre la zona de recolección.
- `OPENING_GRIPPER_START`: abrir gripper antes de bajar.
- `MOVING_TO_PICKUP` (`recoleccion_2`): bajar a la pieza.
- `CLOSING_GRIPPER`: cerrar gripper.
- `MOVING_TO_SAFE_POS_2` (`recoleccion_1`): subir con la pieza.
- `MOVING_TO_BIN`: ir a la caneca correspondiente (`caneca_roja`, etc.).
- `OPENING_GRIPPER_DROP`: abrir gripper para soltar la pieza.
- `RETURNING_TO_HOME_END`: volver a `home` (pose de seguridad).
- `COMPLETED`: registra el fin de la secuencia, envía `joint_command [0,0,0,0]` y vuelve a `IDLE`.

### Lógica importante

- **Arranque de rutina** (`start_sequence`):
  - Ignora nuevas figuras si ya hay una secuencia en curso.
  - No valida trayectorias (asume que `commander` y MoveIt pueden planificar a las poses de `poses.yaml`).
  - Marca `routine_busy := true` al inicio.
- **Secuenciación por tiempo**:
  - Usa `TIME_MOVEMENT` (por defecto 6 s) y `TIME_GRIPPER` (2 s) para espaciar los comandos y dar tiempo a MoveIt/Dynamixel.
  - Utiliza un `create_timer` (`sequence_timer`) que llama periódicamente a `execute_sequence_step` y avanza de estado sólo cuando ha pasado el tiempo configurado.
- **Finalización y HOME real**:
  - Al entrar en `COMPLETED`:
    - Publica logs de éxito.
    - Envía `joint_command [0,0,0,0]` para llevar hombro, codo y muñeca a 0 rad (HOME articular exacto).
    - Publica `routine_busy := false` para que `yolo_recognition_node` pueda rearmar su lógica de detección.

### Parámetros

- `fsm_enabled` (`bool`): actualmente se asume `True` (la lógica de FSM está siempre activa cuando el nodo se ejecuta).
- `TIME_MOVEMENT`, `TIME_GRIPPER`: constantes en el código, pueden ajustarse para adaptarse a la velocidad real del robot.
- Las poses usadas (`home`, `recoleccion_1`, `recoleccion_2`, `caneca_*`) se cargan de `phantomx_pincher_banach_config/poses.yaml`.

### Uso típico

El nodo se lanza normalmente desde `phantomx_pincher.launch.py` con:

```bash
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py \
  use_real_robot:=true \
  start_clasificador:=true
```

En ese caso, el flujo es:

1. `yolo_recognition_node` publica `/figure_type` cuando detecta una figura estable.
2. `clasificador_node` (en `IDLE`) recibe la figura y arranca la FSM.
3. El robot ejecuta la secuencia de pick&place.
4. Al terminar, vuelve a HOME y alinea los joints; luego queda listo para la siguiente figura.
