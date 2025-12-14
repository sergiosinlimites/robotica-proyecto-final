## Paquete `pincher_control`

Este paquete agrupa todos los nodos de control de bajo y alto nivel para el Phantom X Pincher, así como el pipeline de visión basado en YOLOv8 (clasificación).

### Nodos principales

#### 1. `follow_joint_trajectory_node.py` (`follow_joint_trajectory`)

- Nodo Python que conecta los action servers de MoveIt2 con el hardware real (servos Dynamixel AX‑12A).
- Expone dos action servers `FollowJointTrajectory`:
  - `/joint_trajectory_controller/follow_joint_trajectory` → grupo `arm`.
  - `/gripper_trajectory_controller/follow_joint_trajectory` → grupo `gripper`.
- Publica `/joint_states` con las posiciones COMANDADAS de todas las articulaciones.
- Se suscribe a `/set_gripper` (`std_msgs/Bool`) para control directo del gripper:
  - `True` → abrir.
  - `False` → cerrar.
- Hace el mapeo `joint_name → ID` y convierte radianes/metros a ticks de AX‑12A.

#### 2. `clasificador_node.py` (`clasificador_node`)

- Máquina de estados de alto nivel para **pick & place** basado en el tipo de figura detectada.
- Suscripciones:
  - `/figure_type` (`std_msgs/String`): tipos `cubo`, `cilindro`, `pentagono`, `rectangulo`.
- Publicaciones:
  - `/pose_command` (`phantomx_pincher_interfaces/PoseCommand`): órdenes de pose para `commander`.
  - `/set_gripper` (`std_msgs/Bool`): abre/cierra el gripper mediante `follow_joint_trajectory_node`.
  - `/routine_busy` (`std_msgs/Bool`): indica si la FSM está ejecutando una rutina (usado por YOLO para pausar la visión).
  - `/joint_command` (`example_interfaces/Float64MultiArray`): al final de la rutina envía `[0,0,0,0]` para asegurar HOME articular exacto.
- Estados principales (simplificado):
  - HOME inicial → `recoleccion_1` → abrir gripper → `recoleccion_2` → cerrar gripper → retorno a `recoleccion_1` → `caneca_X` → abrir gripper → HOME final.
- Usa tiempos `TIME_MOVEMENT` y `TIME_GRIPPER` para secuenciar los pasos sin saturar MoveIt.
- Al completar la rutina, llama a `send_home_joint_command()` para dejar las 4 articulaciones del brazo en 0 rad.

#### 3. `routine_manager.py` (`routine_manager`)

- Alternativa más genérica al `clasificador_node`, basada en un archivo de rutinas YAML (`config/routines.yaml`).
- Suscripciones:
  - `/figure_type` (`String`): dispara la rutina asociada al nombre de la figura.
  - `/figure_state` (`String`): usado para exigir que haya `vacio` estable antes de repetir la misma rutina.
  - `joint_states` (`sensor_msgs/JointState`): para confirmar si el robot está en HOME.
- Publicaciones:
  - `/pose_command` (`PoseCommand`): igual que `clasificador_node`.
  - `/set_gripper` (`Bool`): control del gripper.
  - `joint_command` (`Float64MultiArray`): para recuperar HOME articular si un paso se queda atascado.
- Ejecuta las rutinas definidas en `config/routines.yaml` paso a paso, con comprobación de llegada mediante TF y tolerancias de distancia.

#### 4. `yolo_recognition_node.py` (`yolo_recognition_node`)

- Nodo de reconocimiento de figuras basado en YOLOv8 **clasificación**.
- Carga el modelo `best.pt` desde:
  - Parámetro `model_path`,
  - o variable de entorno `PINCHER_YOLO_MODEL`,
  - o rutas por defecto dentro de `share/pincher_control/models` o `runs/classify/yolo_shapes`.
- Suscripción:
  - Tópico de imagen (`sensor_msgs/Image`), por defecto `/image_raw` o el dado por `PINCHER_IMAGE_TOPIC`.
  - `/routine_busy` (`Bool`): cuando es `True`, pausa la inferencia para no generar nuevas detecciones durante una rutina.
- Publicaciones:
  - `/figure_type` (`String`): clase confirmada **estable** usando un buffer de últimas detecciones.
  - `/figure_state` (`String`): estado continuo (`cubo`, `cilindro`, `pentagono`, `rectangulo`, `vacio`, `unknown`).
  - `/camera/debug`, `/camera/roi` (`Image`): imagen completa con overlay y recorte del ROI.
- Lógica clave:
  - Define un ROI en la imagen para centrarse en la zona de recolección.
  - Realiza inferencias a frecuencia `inference_hz` (parámetro), menor que la frecuencia de la cámara.
  - Usa un buffer de tamaño fijo para estabilizar la clase antes de publicarla en `/figure_type`.
  - Lleva un `last_published_figure` para no disparar infinitamente la misma figura mientras el objeto no se retira.
  - Cuando el ROI se ve vacío durante varias inferencias o cuando la rutina termina (`/routine_busy` pasa de `True` a `False`), **rear­ma** la detección (vacía el buffer y resetea `last_published_figure`).

#### 5. Otros nodos

- `camera_recognition_node.py`: versión previa/alternativa de reconocimiento (no basada en YOLOv8); se mantiene para referencia.
- `object_sorting_node.py`, `shape_to_pose_node.py`, `pose_search_node.py`, etc.: herramientas auxiliares y prototipos que aprovechan la misma infraestructura de `PoseCommand` y MoveIt.

### Configuración

- `config/routines.yaml`:
  - Define, por figura (`cubo`, `cilindro`, `rectangulo`, `pentagono`), una lista de pasos con:
    - `name`: nombre del paso (Home, Approach, Lift, Place...).
    - `pose`: {x, y, z, roll, pitch, yaw}.
    - `gripper` opcional: `open` / `close`.
  - Usado principalmente por `routine_manager.py`.

- `launch/*.launch.py`:
  - `vision_bringup.launch.py` se encuentra en `phantomx_pincher_bringup`, pero aquí pueden existir lanzadores de utilidad adicionales.

### Entrenamiento de YOLO

El script `phantom_ws/train_yolo.py` (en el nivel superior del workspace) entrena YOLOv8 clasificación usando este paquete sólo como consumidor del modelo entrenado.

Ejemplo de entrenamiento extendido con aumentos:

```bash
cd ~/ros3/KIT_Phantom_X_Pincher_ROS2/phantom_ws

python3 train_yolo.py \
  --base yolov8n-cls.pt \
  --data dataset_yolo \
  --epochs 100 \
  --imgsz 128 \
  --name yolo_shapes_long
```

El modelo resultante (`runs/classify/yolo_shapes_long/weights/best.pt`) puede usarse exportándolo vía `PINCHER_YOLO_MODEL`:

```bash
export PINCHER_YOLO_MODEL=$PWD/runs/classify/yolo_shapes_long/weights/best.pt
```

### Flujo completo de pick & place (resumen)

1. `usb_cam` publica imágenes en `/image_raw`.
2. `yolo_recognition_node` infiere cada `1 / inference_hz` s sobre el ROI y, cuando hay detección estable, publica `/figure_type`.
3. `clasificador_node` recibe `/figure_type` y, si no está ejecutando nada, arranca la FSM:
   - Publica secuencialmente poses a `/pose_command` y órdenes de gripper a `/set_gripper`.
   - Publica `/routine_busy:=true` mientras dura la rutina, de forma que YOLO pausa nuevas inferencias.
4. Al finalizar la rutina:
   - `clasificador_node` envía `joint_command [0,0,0,0]` para garantizar HOME articular exacto.
   - Publica `/routine_busy:=false`, rearmando la lógica de detección en YOLO.
5. El sistema queda listo para detectar y clasificar la siguiente figura.
