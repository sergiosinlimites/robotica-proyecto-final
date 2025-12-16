import cv2
import os
import time

# Configuración
ROI_X_MIN_PCT = 0.45
ROI_X_MAX_PCT = 0.60
ROI_Y_MIN_PCT = 0.62
ROI_Y_MAX_PCT = 0.77

CLASSES = ['cubo', 'rectangulo', 'pentagono', 'cilindro', 'vacio']
DATASET_DIR = "dataset_yolo"

def create_dirs():
    for cls in CLASSES:
        os.makedirs(os.path.join(DATASET_DIR, "train", cls), exist_ok=True)
        os.makedirs(os.path.join(DATASET_DIR, "val", cls), exist_ok=True)

def main():
    create_dirs()
    
    # Intentar abrir la cámara (ajusta el índice si es necesario, 0 o 2)
    cap = cv2.VideoCapture(2) 
    if not cap.isOpened():
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Error: No se pudo abrir la cámara.")
            return

    print("--- CAPTURA DE DATASET YOLO ---")
    print("Teclas para guardar:")
    print("  'c' -> cubo")
    print("  'r' -> rectangulo")
    print("  'p' -> pentagono")
    print("  'l' -> cilindro (l de ciLindro)")
    print("  'v' -> vacio (fondo sin objetos)")
    print("  'q' -> Salir")
    
    count = 0

    while True:
        ret, frame = cap.read()
        if not ret: break

        height, width, _ = frame.shape
        
        # Calcular ROI
        x_min = int(width * ROI_X_MIN_PCT)
        x_max = int(width * ROI_X_MAX_PCT)
        y_min = int(height * ROI_Y_MIN_PCT)
        y_max = int(height * ROI_Y_MAX_PCT)

        # Extraer ROI
        roi = frame[y_min:y_max, x_min:x_max]

        # Dibujar ROI en el frame original
        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
        cv2.putText(frame, "ROI", (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("Captura Dataset", frame)
        cv2.imshow("ROI (Lo que ve la IA)", roi)

        key = cv2.waitKey(1) & 0xFF

        target_class = None
        if key == ord('c'): target_class = 'cubo'
        elif key == ord('r'): target_class = 'rectangulo'
        elif key == ord('p'): target_class = 'pentagono'
        elif key == ord('l'): target_class = 'cilindro'
        elif key == ord('v'): target_class = 'vacio'
        elif key == ord('q'): break

        if target_class:
            # Guardar imagen
            timestamp = int(time.time() * 1000)
            filename = f"{target_class}_{timestamp}.jpg"
            
            # Guardar en train por defecto (luego el usuario puede mover algunas a val si quiere, 
            # o podemos automatizarlo, pero para pocas fotos train está bien para empezar)
            path = os.path.join(DATASET_DIR, "train", target_class, filename)
            cv2.imwrite(path, roi)
            print(f"Guardado: {path}")
            count += 1

    cap.release()
    cv2.destroyAllWindows()
    print(f"Captura finalizada. Total imágenes: {count}")

if __name__ == "__main__":
    main()
