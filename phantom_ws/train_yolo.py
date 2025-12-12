from ultralytics import YOLO

def main():
    # Cargar modelo pre-entrenado (nano classification)
    model = YOLO('yolov8n-cls.pt') 

    # Entrenar el modelo
    # data: ruta a la carpeta que contiene 'train' y 'val'
    # epochs: 20 suele ser suficiente para pocas clases y pocas fotos
    # imgsz: 64 (el ROI es pequeño, no necesitamos 224 o 640)
    results = model.train(data='dataset_yolo', epochs=20, imgsz=64, name='yolo_shapes')

    print("Entrenamiento finalizado.")
    print(f"Mejor modelo guardado en: {results.save_dir}/weights/best.pt")

if __name__ == '__main__':
    main()
