#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
import shutil
from pathlib import Path

from ultralytics import YOLO
from dotenv import load_dotenv, find_dotenv


def load_dotenv_if_present() -> None:
    """Carga .env automáticamente usando python-dotenv."""
    env_path = find_dotenv(usecwd=True)
    if env_path:
        load_dotenv(env_path, override=False)


def main():
    """
    Entrena YOLOv8 clasificación y guarda SIEMPRE en un directorio estable.

    Por defecto:
      project = <phantom_ws>/runs/classify
      name    = yolo_shapes

    Ejemplo para tu caso:
      python3 train_yolo.py --name yolo_shapes3 --export-best /media/sergio/DATOS/TPI2/runs/classify/yolo_shapes3/weights/best.pt
    """
    load_dotenv_if_present()

    parser = argparse.ArgumentParser()
    parser.add_argument("--base", default="yolov8n-cls.pt", help="Modelo base (pre-entrenado)")
    parser.add_argument("--data", default="dataset_yolo", help="Carpeta con train/ y val/")
    parser.add_argument("--epochs", type=int, default=20)
    parser.add_argument("--imgsz", type=int, default=64)
    parser.add_argument("--project", default="", help="Directorio 'project' de Ultralytics (opcional)")
    parser.add_argument("--name", default="yolo_shapes", help="Nombre de la corrida (subcarpeta)")
    parser.add_argument(
        "--export-best",
        default="",
        help="Si se define, copia best.pt a esta ruta (archivo o carpeta).",
    )
    args = parser.parse_args()

    ws_dir = Path(__file__).resolve().parent

    # project por defecto dentro de phantom_ws (estable)
    project = Path(args.project).expanduser() if args.project.strip() else (ws_dir / "runs" / "classify")
    project.mkdir(parents=True, exist_ok=True)

    model = YOLO(args.base)

    results = model.train(
        data=str(Path(args.data).expanduser()),
        epochs=args.epochs,
        imgsz=args.imgsz,
        project=str(project),
        name=args.name,
    )

    best_pt = Path(results.save_dir) / "weights" / "best.pt"
    print("Entrenamiento finalizado.")
    print(f"Mejor modelo guardado en: {best_pt}")

    # Copia opcional del best.pt a una ruta estable que tú elijas
    export = args.export_best.strip()
    if export:
        export_path = Path(export).expanduser()
        if export_path.is_dir():
            export_path = export_path / "best.pt"
        export_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(best_pt, export_path)
        print(f"Copiado best.pt a: {export_path}")


if __name__ == "__main__":
    main()
