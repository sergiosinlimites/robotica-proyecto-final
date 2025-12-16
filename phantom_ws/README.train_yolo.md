## `train_yolo.py`

Script de entrenamiento para modelos **YOLOv8 de clasificación** sobre el dataset `dataset_yolo` (cubo, cilindro, pentágono, rectángulo, vacío). Genera los modelos `best.pt` que luego consume `yolo_recognition_node.py`.

### Rol en el flujo de trabajo

- Crea y entrena un modelo `yolov8*-cls` a partir de un checkpoint base (`yolov8n-cls.pt`, etc.).
- Guarda los resultados en `phantom_ws/runs/classify/<name>/` (pesos, métricas, gráficas).
- Copia opcionalmente el `best.pt` a una ruta fija (`--export-best`), que luego se puede referenciar con `PINCHER_YOLO_MODEL`.

### Argumentos

```python
parser.add_argument("--base",   default="yolov8n-cls.pt", help="Modelo base pre-entrenado")
parser.add_argument("--data",   default="dataset_yolo",   help="Carpeta con train/ y val/")
parser.add_argument("--epochs", type=int, default=20)
parser.add_argument("--imgsz",  type=int, default=64)
parser.add_argument("--project", default="", help="Directorio base de runs/")
parser.add_argument("--name",    default="yolo_shapes",   help="Nombre de la corrida")
parser.add_argument("--export-best", default="", help="Ruta donde copiar best.pt")
```

- `--base`: checkpoint inicial de YOLOv8 clasificación (por ejemplo `yolov8n-cls.pt`).
- `--data`: carpeta que contiene `train/` y `val/` con subcarpetas por clase.
- `--epochs`: número de épocas de entrenamiento (aumenta esto para mejor rendimiento).
- `--imgsz`: tamaño de entrada (por ejemplo 64 o 128).
- `--project`: directorio raíz para las corridas de entrenamiento (por defecto `phantom_ws/runs/classify`).
- `--name`: nombre de la corrida (`runs/classify/<name>`).
- `--export-best`: si se indica, copia el `best.pt` final a esa ruta.

### Flujo interno

1. Carga las variables de entorno desde `.env` si existe (`load_dotenv_if_present`).
2. Calcula la ruta del workspace (`ws_dir = Path(__file__).parent`).
3. Determina el directorio `project` (por defecto `phantom_ws/runs/classify`).
4. Crea el modelo con `YOLO(args.base)`.
5. Llama a `model.train(...)` con:

```python
results = model.train(
    data=str(Path(args.data).expanduser()),
    epochs=args.epochs,
    imgsz=args.imgsz,
    project=str(project),
    name=args.name,
)
```

6. Obtiene la ruta a `best.pt` dentro de `results.save_dir / "weights" / "best.pt"` y la muestra por consola.
7. Si `--export-best` está definido, copia `best.pt` a esa ruta (creando carpetas si es necesario).

### Ejemplos de uso

- **Entrenamiento rápido (20 épocas, 64×64):**

```bash
cd ~/ros3/KIT_Phantom_X_Pincher_ROS2/phantom_ws
python3 train_yolo.py \
  --base yolovasyntetic/yolov8n-cls.pt \
  --data dataset_yolo \
  --epochs 20 \
  --imgsz 64 \
  --name yolo_shapes
```

- **Entrenamiento largo con más resolución (100 épocas, 128×128) y copia del modelo final:**

```bash
python3 train_yolo.py \
  --base yolov8n-cls.pt \
  --data dataset_yolo \
  --epochs 100 \
  --imgsz 128 \
  --name yolo_shapes_long \
  --export-best ../runs/classify/yolo_shapes_long/best.pt
```

- **Usar el modelo entrenado en la visión:**

```bash
export PINCHER_YOLO_MODEL=$PWD/runs/classify/yolo_shapes_long/weights/best.pt
ros2 launch phantomx_pincher_bringup vision_bringup.launch.py ...
```
