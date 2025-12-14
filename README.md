## KIT Phantom X Pincher ROS2

Este repositorio contiene un stack completo en ROS 2 Humble para controlar el robot Phantom X Pincher con MoveIt2 y un pipeline de visión basado en YOLOv8 (clasificación), incluyendo:

- **Bringup de movimiento** (`phantomx_pincher_bringup`): URDF, MoveIt2, controladores (simulación y hardware real) y el nodo `commander` en C++ que envía trayectorias al robot.
- **Control de hardware** (`pincher_control`): nodos para hablar con los Dynamixel, nodos de alto nivel (`clasificador_node`, `routine_manager`) y el nodo de reconocimiento `yolo_recognition_node`.
- **Entrenamiento de YOLO** (`train_yolo.py` y `dataset_yolo`): dataset de figuras (cubo, cilindro, rectángulo, pentágono, vacío) y scripts para entrenar modelos de clasificación.

### Estructura principal

- `phantom_ws/src/phantomx_pincher_description`: URDF/Xacro del robot y entorno (base, canecas, etc.).
- `phantom_ws/src/phantomx_pincher_moveit_config`: configuración de MoveIt2 (SRDF, cinemática, controladores, RViz).
- `phantom_ws/src/phantomx_pincher_bringup`: lanzadores (`phantomx_pincher.launch.py`, `vision_bringup.launch.py`) que levantan todo el stack.
- `phantom_ws/src/pincher_control`: nodos Python de control, visión y rutinas.
- `phantom_ws/train_yolo.py`: script para entrenar YOLOv8 clasificación sobre `dataset_yolo`.

### Cómo compilar

```bash
cd phantom_ws
bash build.sh
source install/setup.bash
```

`build.sh` fuerza el uso de `/usr/bin/python3` (Python 3.10 del sistema) y configura `colcon` para evitar conflictos con entornos `pyenv`.

### Cómo lanzar (robot real + visión + clasificador)

1. **Stack de movimiento (robot real)**

```bash
cd phantom_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py \
  use_real_robot:=true \
  start_clasificador:=true
```

2. **Visión (cámara + YOLO)**

```bash
cd phantom_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

export PINCHER_YOLO_MODEL=$PWD/runs/classify/yolo_shapes_long/weights/best.pt

ros2 launch phantomx_pincher_bringup vision_bringup.launch.py \
  start_camera:=true \
  camera_device:=/dev/video2 \
  start_clasificador:=false \
  inference_hz:=2.0 \
  image_width:=1280 image_height:=720 framerate:=30.0
```

Con esta combinación:

- MoveIt2 y `commander` controlan el brazo real a través del nodo `follow_joint_trajectory`.
- `yolo_recognition_node` detecta figuras en el ROI.
- `clasificador_node` escucha `/figure_type` y ejecuta la rutina de pick&place completa con una máquina de estados, pausando la visión mientras la rutina está en marcha.

Consulta los READMEs específicos en cada paquete para una descripción más detallada.
