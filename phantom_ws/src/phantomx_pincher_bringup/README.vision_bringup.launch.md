## `vision_bringup.launch.py`

Lanzador del **stack de visión** para el Phantom X Pincher.

### Rol en el sistema

- Arranca la cámara (normalmente `usb_cam`) con la resolución y FPS que se indiquen.
- Arranca el nodo `pincher_control/yolo_recognition_node` (YOLOv8 clasificación) conectado al tópico de imagen adecuado.
- Opcionalmente arranca `pincher_control/clasificador_node` y/o `rviz2` para depuración.

### Argumentos principales

- `start_camera` (`true` / `false`): si es `true`, lanza `usb_cam_node_exe`.
- `camera_device`: ruta del dispositivo de vídeo (`/dev/video2`, `/dev/video0`, etc.).
- `image_width`, `image_height`: resolución de la cámara.
- `framerate`: FPS de la cámara.
- `inference_hz`: frecuencia de inferencia de YOLO (Hz). Normalmente menor que el framerate.
- `start_clasificador`: si se pone en `true`, también lanza `clasificador_node` (no necesario si ya se lanzó desde `phantomx_pincher.launch.py`).
- `start_rviz`: si es `true`, arranca `rviz2` para ver cámara y topics de depuración.

### Variables de entorno

- `PINCHER_YOLO_MODEL`: ruta absoluta al `best.pt` de YOLO (clasificador de figuras).
- `PINCHER_IMAGE_TOPIC`: si se define, YOLO leerá de ese tópico en lugar de `/image_raw`.

### Nodos que lanza

- `usb_cam/usb_cam_node_exe` (opcional):
  - Publica `sensor_msgs/Image` en `/image_raw` con la resolución y FPS configurados.
  - Parámetros configurables: `video_device`, `framerate`, `image_width`, `image_height`, etc.
- `pincher_control/yolo_recognition_node` (siempre):
  - Suscribe al tópico de imagen (`image_topic` o `/image_raw`).
  - Carga el modelo desde `model_path` o `PINCHER_YOLO_MODEL`.
  - Publica:
    - `/figure_type` (clase estable: `cubo`, `cilindro`, `pentagono`, `rectangulo`).
    - `/figure_state` (incluye `vacio`, `unknown`).
    - `/camera/debug` y `/camera/roi` para visualización en RViz.
  - Suscribe `/routine_busy` para pausar la inferencia mientras el brazo ejecuta una rutina.
- `pincher_control/clasificador_node` (opcional): FSM de pick&place, ver README específico.
- `rviz2` (opcional): para ver cámara, ROI y otros tópicos.

### Uso típico

```bash
cd ~/ros3/KIT_Phantom_X_Pincher_ROS2/phantom_ws
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
