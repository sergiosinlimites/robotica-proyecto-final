## `routine_manager.py`

Nodo de alto nivel que ejecuta **rutinas predefinidas** (secuencias de poses y acciones de gripper) descritas en `config/routines.yaml`. Es una alternativa más declarativa al `clasificador_node`.

### Rol en el sistema

- Permite definir rutinas de movimiento por nombre (`recoleccion`, `cubo`, `cilindro`, etc.) en YAML, sin codificar la secuencia en Python.
- Se integra con la visión mediante `/figure_type` y `/figure_state`:
  - Dispara automáticamente la rutina asociada a la figura detectada.
  - Evita repetir la misma rutina hasta que el ROI vuelva a `vacio` estable.
- Gestiona el avance paso a paso de cada rutina verificando la llegada a la pose mediante TF.

### Entradas y salidas

- **Suscripciones**:
  - `/figure_type` (`std_msgs/String`): cuando recibe una figura (no vacía), busca en `routines.yaml` una rutina con ese nombre y la ejecuta.
  - `/figure_state` (`std_msgs/String`): usado para saber cuándo el área de recolección está `vacio` de nuevo y permitir repetir una misma rutina.
  - `joint_states` (`sensor_msgs/JointState`): para comprobar si el brazo está en HOME (joints ~0) y para activar la lógica de "ensure_home".
- **Publicaciones**:
  - `/pose_command` (`phantomx_pincher_interfaces/PoseCommand`): envía el siguiente objetivo cartesiano al `commander`.
  - `/set_gripper` (`std_msgs/Bool`): abre/cierra el gripper según el campo `gripper` de cada paso de la rutina.
  - `joint_command` (`example_interfaces/Float64MultiArray`): en caso de timeout, envía `[0,0,0,0]` para recuperar HOME.

### Formato de `config/routines.yaml`

```12:24:phantom_ws/src/pincher_control/config/routines.yaml
routines:
  cubo:
    - name: "Home"
      pose: {x: 0.0, y: 0.0, z: 0.406, roll: 0.0, pitch: 0.0, yaw: 3.142}
      gripper: "open"
    - name: "Approach"
      pose: {x: 0.100, y: 0.0, z: 0.140, roll: 3.142, pitch: 0.195, yaw: 0.000}
    - name: "Lift"
      pose: {x: 0.099, y: 0.0, z: 0.045, roll: 3.142, pitch: 0.194, yaw: 0.000}
      gripper: "close"
    - name: "Approach"
      pose: {x: 0.100, y: 0.0, z: 0.140, roll: 3.142, pitch: 0.195, yaw: 0.000}
    - name: "Place Cubo"
      pose: {x: -0.008, y: 0.118, z: 0.128, roll: 3.142, pitch: 0.087, yaw: 1.635}
      gripper: "open"
    - name: "Home"
      pose: {x: 0.0, y: 0.0, z: 0.406, roll: 0.0, pitch: 0.0, yaw: 3.142}
```

Cada rutina es una lista de pasos; cada paso puede incluir una acción de gripper opcional.

### Lógica de ejecución

- `start_routine_callback` / `figure_callback`:
  - Cargan la lista de pasos de la rutina solicitada.
  - Inicializan `current_step_index = 0`, `is_executing = True` y arrancan el `timer`.
- `execution_loop` (se ejecuta cada 0.1 s):
  1. Comprueba si la rutina ha terminado (`current_step_index >= len(current_routine)`).
  2. Si hay una pausa programada (`pause_until`), espera.
  3. Obtiene el paso actual (`step`) y su `pose`.
  4. Llama a `check_reached(target, tolerance)` usando TF (`phantomx_pincher_base_link` → `phantomx_pincher_end_effector`).
  5. Si la pose se ha alcanzado:
     - Publica la acción de gripper (`set_gripper`) si el paso la define.
     - Avanza al siguiente paso, resetea tiempos, programa una pequeña pausa entre pasos.
  6. Si no se ha alcanzado y ha pasado `step_timeout_sec`, llama a `_recover_to_home()` para recuperar la postura segura.
  7. Si todavía no se ha alcanzado pero no hay timeout, publica (o republica) `pose_command` según `republish_interval_sec`.
- **Gestión de HOME articular**:
  - `ensure_home_active` y `_home_joints_are_zero()` se usan para alinear las articulaciones cuando el brazo está en la pose de HOME pero con joints no exactamente en 0.

### Integración con la visión

- `figure_state_callback`:
  - Marca `vacio_is_stable` cuando se observa `vacio` durante más de `vacio_hold_sec`.
  - Si `require_vacio_between_same_routine` está activo, evita disparar la misma rutina de figura hasta que se detecte `vacio` estable después de completarla.

### Uso típico

Si quieres usar `routine_manager` en lugar de `clasificador_node`:

```bash
# Lanzar movimiento (con o sin real)
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py use_real_robot:=true

# Lanzar visión (solo YOLO + cámara)
ros2 launch phantomx_pincher_bringup vision_bringup.launch.py \
  start_camera:=true \
  start_clasificador:=false ...

# Lanzar routine_manager
ros2 run pincher_control routine_manager
```

Cuando `yolo_recognition_node` publique `/figure_type: "cubo"`, `routine_manager` ejecutará la rutina `cubo` definida en `routines.yaml`.
