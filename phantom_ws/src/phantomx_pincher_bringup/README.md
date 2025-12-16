## Paquete `phantomx_pincher_bringup`

Este paquete contiene los lanzadores y la configuración necesaria para levantar el stack de movimiento del Phantom X Pincher con MoveIt 2, tanto en simulación (ros2_control fake) como sobre el robot real usando `pincher_control/follow_joint_trajectory`.

### Lanzadores principales

#### `phantomx_pincher.launch.py`

- **Argumentos**:
  - `use_real_robot` (`true`/`false`):
    - `false` → usa `controller_manager` + `ros2_control_node` + controladores fake (`joint_trajectory_controller`, `gripper_trajectory_controller`).
    - `true`  → usa el nodo `pincher_control/follow_joint_trajectory` para hablar con los Dynamixel reales.
  - `start_clasificador` (`true`/`false`):
    - Si es `true`, lanza el nodo `pincher_control/clasificador_node` como máquina de estados de pick&place.

- **Nodos comunes** (sim y real):
  - `robot_state_publisher`: publica el URDF del robot (`phantomx_pincher.urdf.xacro`).
  - `commander` (C++): wrapper de `MoveGroupInterface` que suscribe:
    - `/pose_command` (`phantomx_pincher_interfaces/PoseCommand`) → planea y ejecuta trayectorias cartesiano/articulares.
    - `/joint_command` (`Float64MultiArray`) → mueve directamente las 4 articulaciones del brazo a los valores dados.
    - `/open_gripper` (`Bool`) → abre/cierra el gripper vía MoveIt.

- **Simulación (`use_real_robot:=false`)**:
  - `controller_manager/ros2_control_node` con `controllers_position.yaml`.
  - Spawners para `joint_state_broadcaster`, `joint_trajectory_controller`, `gripper_trajectory_controller`.
  - `move_group.launch.py` de MoveIt2 en modo sim (`ros2_control_plugin=fake`, `manage_controllers=false`, `enable_rviz=true`).

- **Robot real (`use_real_robot:=true`)**:
  - `pincher_control/follow_joint_trajectory`:
    - Expone los action servers
      - `/joint_trajectory_controller/follow_joint_trajectory` (brazo)
      - `/gripper_trajectory_controller/follow_joint_trajectory` (gripper)
    - Controla directamente los servos Dynamixel (IDs, velocidad, torque…).
  - `move_group.launch.py` se arranca en modo real (`ros2_control=false`, `ros2_control_plugin=real`, `manage_controllers=false`).

- **Clasificador con máquina de estados** (`start_clasificador:=true`):
  - Lanza `pincher_control/clasificador_node`:
    - Escucha `/figure_type` (salida estable del nodo YOLO).
    - Publica `/pose_command` y `/set_gripper` para ejecutar una secuencia de pick&place completa.
    - Publica `/routine_busy` para pausar la visión mientras la rutina está en marcha.

#### `vision_bringup.launch.py`

Bringup de la cámara y del nodo de reconocimiento YOLO.

- **Argumentos**:
  - `start_camera` (`true`/`false`): arranca o no el nodo `usb_cam`.
  - `camera_device`: dispositivo de vídeo (`/dev/video2`, `/dev/video0`, etc.).
  - `image_width`, `image_height`: resolución deseada (p. ej. 1280×720).
  - `framerate`: FPS de la cámara (p. ej. 30.0).
  - `inference_hz`: frecuencia de inferencia de YOLO (Hz); suele ser menor que el FPS de cámara.
  - `start_clasificador`: opcional, permite lanzar `clasificador_node` desde aquí (no recomendado si ya se lanza en `phantomx_pincher.launch.py`).
  - `start_rviz`: arranca RViz2 para depuración visual rápida.

- **Uso de variables de entorno**:
  - `PINCHER_YOLO_MODEL`: ruta al `best.pt` de YOLO (clasificación).
  - `PINCHER_IMAGE_TOPIC`: tópico de imagen a usar (si no se define, YOLO suscribe a `/image_raw`).

- **Nodos**:
  - `usb_cam/usb_cam_node_exe`:
    - Publica imágenes en `/image_raw` con la resolución/FPS configurados.
  - `pincher_control/yolo_recognition_node`:
    - Carga el modelo de clasificación YOLOv8.
    - Se suscribe al tópico de imagen (`PINCHER_IMAGE_TOPIC` o `/image_raw`).
    - Publica:
      - `/figure_type` (`String`): clase confirmada (cubo, cilindro, pentagono, rectangulo), sólo cuando la detección es estable.
      - `/figure_state` (`String`): estado continuo (incluye `vacio`, `unknown`).
      - `/camera/debug`, `/camera/roi` (`Image`): vistas para depuración en RViz.
    - Se suscribe a `/routine_busy` para pausar la inferencia mientras el brazo ejecuta una rutina.
  - `pincher_control/clasificador_node` (opcional): si se lanza desde aquí, actúa igual que desde `phantomx_pincher.launch.py`.
  - `rviz2` (opcional): para visualizar cámara/ROI y topics de depuración.

### Combinaciones típicas de lanzamiento

- **Simulación completa con FSM:**

```bash
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py \
  use_real_robot:=false \
  start_clasificador:=true

ros2 launch phantomx_pincher_bringup vision_bringup.launch.py \
  start_camera:=true \
  start_clasificador:=false \
  inference_hz:=2.0 \
  image_width:=1280 image_height:=720 framerate:=30.0
```

- **Robot real con visión y clasificación:**

```bash
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py \
  use_real_robot:=true \
  start_clasificador:=true

ros2 launch phantomx_pincher_bringup vision_bringup.launch.py \
  start_camera:=true \
  camera_device:=/dev/video2 \
  start_clasificador:=false \
  inference_hz:=2.0 \
  image_width:=1280 image_height:=720 framerate:=30.0
```
