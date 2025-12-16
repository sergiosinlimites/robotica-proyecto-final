#!/usr/bin/env python3

"""
Nodo de reconocimiento de figuras geométricas con ROI específico.
Usa Hu Moments (cv2.matchShapes) para comparar con figuras ideales.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import math

class CameraRecognitionNode(Node):
    def __init__(self):
        super().__init__('camera_recognition_node')

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

        # Buffer para estabilizar la detección
        self.detection_buffer = []
        self.buffer_size = 10
        self.last_published_figure = ""

        # Generar formas de referencia para comparar
        self.reference_shapes = self.generate_reference_shapes()

        self.get_logger().info("Camera Recognition Node Initialized (Hu Moments)")
        self.get_logger().info(f"ROI X: {self.roi_x_min_pct}-{self.roi_x_max_pct}, Y: {self.roi_y_min_pct}-{self.roi_y_max_pct}")

    def generate_reference_shapes(self):
        """Genera contornos ideales para comparar."""
        shapes = {}
        
        # 1. Cubo (Cuadrado)
        shapes['cubo'] = np.array([[[0,0]], [[0,100]], [[100,100]], [[100,0]]], dtype=np.int32)
        
        # 2. Rectángulo (Relación 2:1)
        shapes['rectangulo'] = np.array([[[0,0]], [[0,100]], [[200,100]], [[200,0]]], dtype=np.int32)
        
        # 3. Pentágono (Regular)
        pts = []
        for i in range(5):
            ang = 2 * math.pi * i / 5
            pts.append([[int(100 + 100 * math.cos(ang)), int(100 + 100 * math.sin(ang))]])
        shapes['pentagono'] = np.array(pts, dtype=np.int32)

        # 4. Cilindro (Círculo)
        pts = []
        for i in range(30):
            ang = 2 * math.pi * i / 30
            pts.append([[int(100 + 100 * math.cos(ang)), int(100 + 100 * math.sin(ang))]])
        shapes['cilindro'] = np.array(pts, dtype=np.int32)
        
        return shapes

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        height, width, _ = cv_image.shape

        # Calcular coordenadas del ROI
        x_min = int(width * self.roi_x_min_pct)
        x_max = int(width * self.roi_x_max_pct)
        y_min = int(height * self.roi_y_min_pct)
        y_max = int(height * self.roi_y_max_pct)

        # Extraer ROI
        roi = cv_image[y_min:y_max, x_min:x_max]
        
        # Procesar ROI
        detected_shape, score = self.process_roi(roi)

        # Dibujar ROI en la imagen original para debug
        color = (0, 255, 0) if detected_shape else (0, 0, 255)
        cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), color, 2)
        
        if detected_shape:
            label = f"{detected_shape} ({score:.3f})"
            cv2.putText(cv_image, label, (x_min, y_min - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            self.update_buffer(detected_shape)
        else:
            self.update_buffer(None)

        # Publicar imagen de debug
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            self.get_logger().error(f"Error publishing debug image: {e}")

    def process_roi(self, roi_img):
        """Procesa la imagen recortada (ROI) usando la lógica sugerida por el usuario."""
        gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        
        # Usamos adaptiveThreshold que es más robusto que el fijo de 220
        # pero mantenemos la lógica de contornos del usuario
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                     cv2.THRESH_BINARY_INV, 11, 2)
        
        # Limpieza de ruido
        kernel = np.ones((3,3), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_shape = None
        max_area = 0
        min_score = 0.0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 1000: # Filtrar ruido pequeño
                continue

            # Lógica del usuario: epsilon = 0.01 * arcLength
            epsilon = 0.01 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            vertices = len(approx)
            
            # Relación de aspecto para diferenciar cuadrado/rectángulo
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = float(w) / h

            self.get_logger().info(f"Shape: Vertices={vertices}, Area={area:.0f}, AR={aspect_ratio:.3f}")

            shape_found = "unknown"

            if vertices == 3:
                shape_found = "triangulo"
            elif vertices == 4:
                if 0.90 <= aspect_ratio <= 1.10:
                    shape_found = "cubo"
                else:
                    shape_found = "rectangulo"
            elif vertices == 5:
                shape_found = "pentagono"
            elif vertices == 6:
                shape_found = "hexagono"
            else:
                # Si tiene muchos vértices, verificamos circularidad
                # Círculo perfecto = 1.0. Cuadrado ~0.78.
                peri = cv2.arcLength(contour, True)
                if peri > 0:
                    circularity = 4 * np.pi * (area / (peri * peri))
                    self.get_logger().info(f"   -> Circularity: {circularity:.3f}")
                    
                    if circularity > 0.85:
                        shape_found = "cilindro"
                    else:
                        # Si tiene muchos vértices pero es "cuadrado" (circularidad baja)
                        # Probablemente es un cuadrado con ruido
                        if 0.90 <= aspect_ratio <= 1.10:
                            shape_found = "cubo"
                        else:
                            shape_found = "unknown" # Ruido

            if shape_found != "unknown" and area > max_area:
                max_area = area
                best_shape = shape_found

        return best_shape, min_score

    def update_buffer(self, shape):
        """Estabiliza la detección usando un buffer."""
        if shape:
            self.detection_buffer.append(shape)
        else:
            # Si no hay shape, limpiamos gradualmente o reseteamos
            if self.detection_buffer:
                self.detection_buffer.pop(0)
        
        if len(self.detection_buffer) > self.buffer_size:
            self.detection_buffer.pop(0)

        # Si el buffer está lleno y consistente
        if len(self.detection_buffer) >= self.buffer_size * 0.8: # 80% de coincidencia
            # Encontrar el elemento más común
            from collections import Counter
            c = Counter(self.detection_buffer)
            most_common, count = c.most_common(1)[0]
            
            if count >= self.buffer_size * 0.8:
                if most_common != self.last_published_figure:
                    self.get_logger().info(f"Figura confirmada: {most_common}")
                    msg = String()
                    msg.data = most_common
                    self.figure_pub.publish(msg)
                    self.last_published_figure = most_common

def main(args=None):
    rclpy.init(args=args)
    node = CameraRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
