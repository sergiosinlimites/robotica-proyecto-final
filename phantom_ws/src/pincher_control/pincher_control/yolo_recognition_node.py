#!/usr/bin/env python3

"""
Nodo de reconocimiento de figuras usando YOLOv8 (Clasificación).
"""

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import os

class YoloRecognitionNode(Node):
    def __init__(self):
        super().__init__('yolo_recognition_node')

        # Suscripción a la cámara
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        # Publicadores
        self.figure_pub = self.create_publisher(String, '/figure_type', 10)
        self.debug_pub = self.create_publisher(Image, '/camera/debug', 10)

        self.bridge = CvBridge()

        # Configuración del ROI (porcentajes)
        self.roi_x_min_pct = 0.45
        self.roi_x_max_pct = 0.60
        self.roi_y_min_pct = 0.62
        self.roi_y_max_pct = 0.77

        # Cargar Modelo YOLO
        # Buscamos el modelo en la ruta donde se guarda tras el entrenamiento
        # Ojo: El usuario debe haber entrenado primero.
        # Si no existe, usaremos un placeholder o fallaremos elegantemente.
        model_path = '/home/sergio/ros3/KIT_Phantom_X_Pincher_ROS2/phantom_ws/runs/classify/yolo_shapes/weights/best.pt'
        
        self.model = None
        if os.path.exists(model_path):
            self.get_logger().info(f"Cargando modelo YOLO desde: {model_path}")
            self.model = YOLO(model_path)
        else:
            self.get_logger().warn(f"No se encontró modelo en {model_path}. Esperando entrenamiento...")

        # Control de frecuencia (1Hz)
        self.last_inference_time = 0.0
        self.inference_interval = 1.0 # segundos

        self.get_logger().info("YOLO Recognition Node Initialized (1Hz)")

    def image_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Solo procesar si ha pasado el tiempo suficiente
        if current_time - self.last_inference_time < self.inference_interval:
            # Aún así publicamos la imagen de debug para ver el video fluido, 
            # pero sin correr inferencia nueva (usamos la última detección)
            # O simplemente retornamos si queremos ahorrar CPU total.
            # Para UX, mejor mostrar video fluido.
            pass
        else:
            self.last_inference_time = current_time
            self.run_inference(msg)

    def run_inference(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        height, width, _ = cv_image.shape

        # Calcular ROI
        x_min = int(width * self.roi_x_min_pct)
        x_max = int(width * self.roi_x_max_pct)
        y_min = int(height * self.roi_y_min_pct)
        y_max = int(height * self.roi_y_max_pct)

        # Extraer ROI
        roi = cv_image[y_min:y_max, x_min:x_max]
        
        detected_class = "unknown"
        confidence = 0.0

        if self.model:
            # Inferencia YOLO
            # verbose=False para no llenar la terminal
            results = self.model(roi, verbose=False) 
            
            # Obtener la clase con mayor probabilidad
            # results[0].probs.top1 es el índice
            # results[0].names es el diccionario de nombres
            top1_index = results[0].probs.top1
            confidence = results[0].probs.top1conf.item()
            
            if confidence > 0.7: # Umbral de confianza
                detected_class = results[0].names[top1_index]

        # Dibujar ROI y texto
        color = (0, 255, 0) if detected_class != "unknown" and detected_class != "vacio" else (0, 0, 255)
        cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), color, 2)
        
        label = f"{detected_class} ({confidence:.2f})"
        cv2.putText(cv_image, label, (x_min, y_min - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Publicar imagen de debug
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            self.get_logger().error(f"Error publishing debug image: {e}")

        # Lógica de publicación (solo si no es vacio y es estable)
        if detected_class != "unknown" and detected_class != "vacio":
            self.update_buffer(detected_class)
        else:
            self.detection_buffer = [] # Reset si pierde el objeto

    def update_buffer(self, shape):
        self.detection_buffer.append(shape)
        if len(self.detection_buffer) > self.buffer_size:
            self.detection_buffer.pop(0)

        if len(self.detection_buffer) == self.buffer_size:
            if all(s == shape for s in self.detection_buffer):
                if shape != self.last_published_figure:
                    self.get_logger().info(f"Figura confirmada (YOLO): {shape}")
                    msg = String()
                    msg.data = shape
                    self.figure_pub.publish(msg)
                    self.last_published_figure = shape

def main(args=None):
    rclpy.init(args=args)
    node = YoloRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
