#!/usr/bin/env python3

"""
Verifica un modelo YOLOv8 de CLASIFICACIÓN (Ultralytics) contra un dataset tipo ImageNet:

  dataset_yolo/
    val/
      cubo/
      rectangulo/
      pentagono/
      cilindro/
      vacio/

Uso:
  python3 verify_yolo.py --model /ruta/a/best.pt --data dataset_yolo/val

Salida:
  - Accuracy top-1 (raw)
  - Accuracy top-1 con umbral (predicciones bajo umbral -> 'unknown')
  - Matriz de confusión (raw)
  - Ejemplos de errores (opcional)
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from pathlib import Path

from ultralytics import YOLO
from dotenv import load_dotenv, find_dotenv

try:
    import cv2  # opcional (solo si usas --image con --roi)
except Exception:  # pragma: no cover
    cv2 = None


IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}

def load_dotenv_if_present() -> None:
    """Carga .env automáticamente usando python-dotenv."""
    env_path = find_dotenv(usecwd=True)
    if env_path:
        load_dotenv(env_path, override=False)


def resolve_model_path(p: Path) -> Path:
    """Permite pasar:
    - archivo .pt directo
    - directorio de corrida (p.ej. .../runs/classify/yolo_shapes3) -> weights/best.pt
    - directorio weights -> best.pt
    """
    if p.is_file():
        return p

    if p.is_dir():
        # caso: .../yolo_shapes3
        best = p / "weights" / "best.pt"
        if best.exists():
            return best

        # caso: .../weights
        best = p / "best.pt"
        if best.exists():
            return best

        # fallback: last.pt
        last = p / "weights" / "last.pt"
        if last.exists():
            return last
        last = p / "last.pt"
        if last.exists():
            return last

    return p


def iter_images_by_class(val_dir: Path):
    for class_dir in sorted([p for p in val_dir.iterdir() if p.is_dir()]):
        cls = class_dir.name
        for img_path in sorted(class_dir.rglob("*")):
            if img_path.is_file() and img_path.suffix.lower() in IMG_EXTS:
                yield cls, img_path


def predict_one(
    model: YOLO,
    img_path: Path,
    conf_thr: float,
    use_roi: bool,
    roi_x_min_pct: float,
    roi_x_max_pct: float,
    roi_y_min_pct: float,
    roi_y_max_pct: float,
    force_roi: bool,
):
    if use_roi:
        if cv2 is None:
            raise SystemExit("OpenCV no disponible. Instala opencv-python o ejecuta sin --roi.")
        img = cv2.imread(str(img_path))
        if img is None:
            raise SystemExit(f"No se pudo leer imagen: {img_path}")
        h, w = img.shape[:2]

        # Heurística: imágenes de dataset_yolo suelen ser ROI ya recortado (p.ej. 96x72).
        # Aplicar ROI de nuevo produce un crop minúsculo casi todo fondo → predice 'vacio'.
        if not force_roi and min(h, w) < 200:
            print(
                "[WARN] La imagen parece ya ser un ROI recortado (muy pequeña). "
                "Ignorando --roi para evitar doble recorte. "
                "Si quieres forzar el recorte igualmente, agrega --force-roi."
            )
            res = model(str(img_path), verbose=False)[0]
        else:
            x_min = int(w * roi_x_min_pct)
            x_max = int(w * roi_x_max_pct)
            y_min = int(h * roi_y_min_pct)
            y_max = int(h * roi_y_max_pct)
            roi = img[y_min:y_max, x_min:x_max]
            res = model(roi, verbose=False)[0]
    else:
        res = model(str(img_path), verbose=False)[0]

    probs = res.probs
    if probs is None:
        raise SystemExit("El resultado no trae 'probs'. ¿Seguro que el modelo es de clasificación?")

    names = model.names
    idx_to_name = {int(k): v for k, v in names.items()} if isinstance(names, dict) else dict(enumerate(names))

    top1 = int(probs.top1)
    conf = float(probs.top1conf.item())
    pred_raw = idx_to_name.get(top1, str(top1))
    pred_thr = pred_raw if conf >= conf_thr else "unknown"
    return pred_raw, pred_thr, conf


def main():
    load_dotenv_if_present()

    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True, help="Ruta a best.pt (YOLOv8 classify)")
    parser.add_argument("--data", default="dataset_yolo/val", help="Ruta a dataset val/")
    parser.add_argument("--image", default="", help="Ruta a UNA imagen para validar (modo single-image)")
    parser.add_argument("--conf", type=float, default=0.70, help="Umbral de confianza para marcar 'unknown'")
    parser.add_argument("--max-errors", type=int, default=20, help="Máximo de errores a imprimir")
    parser.add_argument("--roi", action="store_true", help="En modo --image, recorta ROI como el nodo")
    parser.add_argument("--force-roi", action="store_true", help="Forzar recorte ROI incluso si la imagen es pequeña")
    parser.add_argument("--roi-x-min", type=float, default=0.45)
    parser.add_argument("--roi-x-max", type=float, default=0.60)
    parser.add_argument("--roi-y-min", type=float, default=0.62)
    parser.add_argument("--roi-y-max", type=float, default=0.77)
    args = parser.parse_args()

    model_path = resolve_model_path(Path(args.model).expanduser())

    if not model_path.exists():
        raise SystemExit(f"No existe el modelo: {model_path}")

    model = YOLO(str(model_path))
    image_arg = args.image.strip()

    # -----------------------
    # Modo: validar 1 imagen
    # -----------------------
    if image_arg:
        img_path = Path(image_arg).expanduser()
        if not img_path.exists():
            raise SystemExit(f"No existe la imagen: {img_path}")

        pred_raw, pred_thr, conf = predict_one(
            model=model,
            img_path=img_path,
            conf_thr=args.conf,
            use_roi=bool(args.roi),
            roi_x_min_pct=args.roi_x_min,
            roi_x_max_pct=args.roi_x_max,
            roi_y_min_pct=args.roi_y_min,
            roi_y_max_pct=args.roi_y_max,
            force_roi=bool(args.force_roi),
        )

        print("--- SINGLE IMAGE ---")
        print(f"imagen: {img_path}")
        print(f"pred_raw: {pred_raw} | conf: {conf:.4f}")
        print(f"pred_thr (thr={args.conf:.2f}): {pred_thr}")
        return

    # -----------------------
    # Modo: evaluar dataset
    # -----------------------
    val_dir = Path(args.data).expanduser()
    if not val_dir.exists():
        raise SystemExit(f"No existe el directorio val: {val_dir}")

    names = model.names  # dict idx->name
    idx_to_name = {int(k): v for k, v in names.items()} if isinstance(names, dict) else dict(enumerate(names))

    # Clases esperadas (a partir del folder)
    classes = sorted([p.name for p in val_dir.iterdir() if p.is_dir()])
    if not classes:
        raise SystemExit(f"No hay subdirectorios de clases en: {val_dir}")

    # Confusion matrix raw: true -> pred -> count
    cm_raw = defaultdict(lambda: defaultdict(int))
    cm_thr = defaultdict(lambda: defaultdict(int))

    total = 0
    correct_raw = 0
    correct_thr = 0
    shown_errors = 0

    for true_cls, img_path in iter_images_by_class(val_dir):
        total += 1
        res = model(str(img_path), verbose=False)[0]
        probs = res.probs

        if probs is None:
            raise SystemExit("El resultado no trae 'probs'. ¿Seguro que el modelo es de clasificación?")

        top1 = int(probs.top1)
        conf = float(probs.top1conf.item())
        pred_raw = idx_to_name.get(top1, str(top1))

        pred_thr = pred_raw if conf >= args.conf else "unknown"

        cm_raw[true_cls][pred_raw] += 1
        cm_thr[true_cls][pred_thr] += 1

        if pred_raw == true_cls:
            correct_raw += 1
        if pred_thr == true_cls:
            correct_thr += 1

        if pred_raw != true_cls and shown_errors < args.max_errors:
            print(f"[ERR] {img_path} | true={true_cls} pred={pred_raw} conf={conf:.3f}")
            shown_errors += 1

    def print_cm(cm, title: str):
        labels = sorted({*classes, *{p for d in cm.values() for p in d.keys()}, "unknown"} - {"unknown"} )  # ensure stable
        # Para raw normalmente no aparece unknown; para thr sí.
        if any("unknown" in cm[t] for t in cm.keys()):
            labels = labels + ["unknown"]
        print("\n" + title)
        print("true\\pred," + ",".join(labels))
        for t in classes:
            row = [str(cm[t].get(p, 0)) for p in labels]
            print(f"{t}," + ",".join(row))

    acc_raw = correct_raw / total if total else 0.0
    acc_thr = correct_thr / total if total else 0.0

    print("\n--- RESULTADOS ---")
    print(f"Total imágenes: {total}")
    print(f"Accuracy top-1 (raw): {acc_raw:.4f}")
    print(f"Accuracy top-1 (thr={args.conf:.2f} -> unknown): {acc_thr:.4f}")
    print(f"Clases (carpetas): {classes}")
    print(f"Clases (modelo): {list(idx_to_name.values())}")

    print_cm(cm_raw, title="--- Matriz de confusión (raw) ---")
    print_cm(cm_thr, title=f"--- Matriz de confusión (thr={args.conf:.2f}) ---")


if __name__ == "__main__":
    main()

