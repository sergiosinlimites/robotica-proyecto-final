import os
import shutil
import random

DATASET_DIR = "dataset_yolo"
CLASSES = ['cubo', 'rectangulo', 'pentagono', 'cilindro', 'vacio']
VAL_SPLIT = 0.2  # 20% para validación

def main():
    print("--- DIVIDIENDO DATASET (TRAIN -> VAL) ---")
    
    for cls in CLASSES:
        train_path = os.path.join(DATASET_DIR, "train", cls)
        val_path = os.path.join(DATASET_DIR, "val", cls)
        
        # Crear directorios si no existen
        os.makedirs(val_path, exist_ok=True)
        
        # Listar imágenes en train
        images = [f for f in os.listdir(train_path) if f.endswith('.jpg')]
        random.shuffle(images)
        
        num_val = int(len(images) * VAL_SPLIT)
        images_to_move = images[:num_val]
        
        print(f"Clase '{cls}': Total {len(images)} -> Moviendo {num_val} a val")
        
        for img in images_to_move:
            src = os.path.join(train_path, img)
            dst = os.path.join(val_path, img)
            shutil.move(src, dst)
            
    print("División completada.")

if __name__ == "__main__":
    main()
