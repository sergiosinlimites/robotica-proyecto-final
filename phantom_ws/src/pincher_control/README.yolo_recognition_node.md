## `yolo_recognition_node.py`

Nodo de reconocimiento de figuras basado en **YOLOv8 clasificación**. Encapsula la inferencia de un modelo `yolov8*-cls` entrenado con el dataset `dataset_yolo` (cubo, cilindro, pentágono, rectángulo, vacío).

### Rol en el sistema

- Suscribe al flujo de vídeo de la cámara (normalmente `/image_raw`).
- Recorta una región de interés (ROI) centrada en la zona de recolección sobre la mesa.
- Ejecuta inferencias de clasificación sobre el ROI a una frecuencia controlada (`inference_hz`).
- Publica la clase detectada:
  - **`/figure_type`**: sólo cuando la detección es estable (mismo resultado en N frames) y distinta de `vacio/unknown`.
  - **`/figure_state`**: estado continuo (`cubo`, `cilindro`, `pentagono`, `rectangulo`, `vacio`, `unknown`).
- Publica imágenes de depuración (`/camera/debug`, `/camera/roi`).
- Se sincroniza con la FSM de pick&place a través de `/routine_busy` para no disparar nuevas detecciones mientras el brazo está en movimiento.

### Entradas y salidas

- **Suscripciones**:
  - `image_topic` (parámetro) o `PINCHER_IMAGE_TOPIC` / `/image_raw` (`sensor_msgs/Image`).
  - `/routine_busy` (`std_msgs/Bool`):
    - `True`  → pausa la inferencia (no se actualiza el buffer de detección).
    - `False` → reanuda la inferencia y **rearma** la lógica de publicación para permitir nuevas detecciones.
- **Publicaciones**:
  - `/figure_type` (`std_msgs/String`): clase estable distinta de `vacio`/`unknown` (ej. `"cubo"`).
  - `/figure_state` (`std_msgs/String`): clase "raw" (top1) o `unknown` si la confianza < `confidence_threshold`.
  - `/camera/debug` (`sensor_msgs/Image`): imagen completa con el ROI dibujado y texto con la clase/confianza.
  - `/camera/roi` (`sensor_msgs/Image`): sólo el recorte de la ROI.

### Parámetros clave

- `model_path` (`str`): ruta al `.pt` de YOLOv8 (clasificación). Si está vacío, el nodo intenta buscar:
  - `PINCHER_YOLO_MODEL` (entorno),
  - `share/pincher_control/models/best.pt`,
  - `runs/classify/yolo_shapes/weights/best.pt`, etc.
- `image_topic` (`str`): tópico de imagen; si está vacío, se usa `PINCHER_IMAGE_TOPIC` o `/image_raw`.
- `confidence_threshold` (`float`): confianza mínima para aceptar el `top1` como clase válida (`detected_class`), por defecto `0.7`.
- `inference_hz` (`float`): frecuencia de inferencia (ej. `2.0` Hz).
- `publish_roi` (`bool`): si es `True`, publica también el ROI en `/camera/roi`.
- ROI (porcentaje de la imagen): `roi_x_min_pct`, `roi_x_max_pct`, `roi_y_min_pct`, `roi_y_max_pct`.

### Lógica de detección y rearmado

- Cada vez que llega una imagen:
  1. Se recorta el ROI definido por los porcentajes X/Y.
  2. Si `vision_enabled` es `False` (rutina en curso), sólo se publican imágenes de debug/ROI, **sin inferir**.
  3. Si ha pasado más de `1/inference_hz` segundos desde la última inferencia, se llama a `self.model(roi)` y se obtiene la clase top1.
  4. Se calcula `detected_class` (sólo si `top1conf >= confidence_threshold`) y se actualiza un buffer de tamaño `buffer_size` (por defecto 10).
  5. Cuando los últimos `buffer_size` elementos del buffer son iguales y distintos de `"vacio"/"unknown"`, y diferentes de `last_published_figure`, se publica en `/figure_type` y se actualiza `last_published_figure`.
- **Rearmado**:
  - Cuando `figure_state` pasa a `vacio` varias veces seguidas (`vacio_streak >= vacio_reset_count`), se resetea `last_published_figure`.
  - Cuando `/routine_busy` cambia de `True` a `False` (rutina terminada), se limpian:

```331:0:phantom_ws/src/pincher_control/pincher_control/yolo_recognition_node.py
self.detection_buffer = []
self.last_published_figure = ""
self.vacio_streak = 0
```

  - Esto garantiza que **un segundo objeto de la misma clase** pueda volver a disparar una nueva publicación en `/figure_type` inmediatamente después de terminar la rutina.

### Uso típico

Este nodo se lanza siempre desde `vision_bringup.launch.py`, por ejemplo:

```bash
ros2 launch phantomx_pincher_bringup vision_bringup.launch.py \
  start_camera:=true \
  camera_device:=/dev/video2 \
  inference_hz:=2.0 \
  image_width:=1280 image_height:=720 framerate:=30.0
```

Antes de lanzar, exporta la ruta al modelo:

```bash
export PINCHER_YOLO_MODEL=$PWD/runs/classify/yolo_shapes_long/weights/best.pt
```
