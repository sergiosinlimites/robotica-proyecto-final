#!/usr/bin/env python3

"""Nodo de reconocimiento de figuras usando YOLOv8 (Clasificación)."""

import os
from pathlib import Path

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from dotenv import find_dotenv, load_dotenv
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from ultralytics import YOLO


class YoloRecognitionNode(Node):
    def __init__(self):
        super().__init__("yolo_recognition_node")

        self._load_dotenv_if_present()

        # ------------------------
        # Parámetros ROS
        # ------------------------
        # Si queda vacío, tomamos PINCHER_IMAGE_TOPIC o /image_raw
        self.declare_parameter("image_topic", "")
        self.declare_parameter("model_path", "")
        self.declare_parameter("confidence_threshold", 0.7)
        self.declare_parameter("inference_hz", 1.0)
        self.declare_parameter("publish_roi", True)

        self.declare_parameter("roi_x_min_pct", 0.45)
        self.declare_parameter("roi_x_max_pct", 0.60)
        self.declare_parameter("roi_y_min_pct", 0.62)
        self.declare_parameter("roi_y_max_pct", 0.77)

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value.strip()
        if not image_topic:
            image_topic = os.environ.get("PINCHER_IMAGE_TOPIC", "/image_raw").strip() or "/image_raw"

        # Suscripción a la cámara
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)

        # Publicadores
        # - /figure_type: solo se publica cuando hay detección estable (NO vacio/unknown)
        # - /figure_state: estado continuo (incluye vacio/unknown) para gating del ejecutor de rutinas
        self.figure_pub = self.create_publisher(String, "/figure_type", 10)
        self.figure_state_pub = self.create_publisher(String, "/figure_state", 10)
        self.debug_pub = self.create_publisher(Image, "/camera/debug", 10)
        self.roi_pub = self.create_publisher(Image, "/camera/roi", 10)

        # Permite pausar la detección mientras una rutina está en ejecución
        # (clasificador_node publica /routine_busy=True durante la secuencia).
        self.vision_enabled = True
        self.busy_sub = self.create_subscription(
            Bool,
            "/routine_busy",
            self.busy_callback,
            10,
        )

        self.bridge = CvBridge()

        # Configuración del ROI (porcentajes)
        self.roi_x_min_pct = float(self.get_parameter("roi_x_min_pct").value)
        self.roi_x_max_pct = float(self.get_parameter("roi_x_max_pct").value)
        self.roi_y_min_pct = float(self.get_parameter("roi_y_min_pct").value)
        self.roi_y_max_pct = float(self.get_parameter("roi_y_max_pct").value)

        # Buffer para estabilizar la detección (evita falsos positivos)
        self.detection_buffer = []
        self.buffer_size = 10
        self.last_published_figure = ""
        self.vacio_streak = 0
        self.vacio_reset_count = 3  # inferencias seguidas en "vacio" para rearmar detección

        # ------------------------
        # Cargar Modelo YOLO (clasificación)
        # ------------------------
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.publish_roi = bool(self.get_parameter("publish_roi").value)

        requested_model_path = self.get_parameter("model_path").get_parameter_value().string_value.strip()
        model_path = self._resolve_model_path(requested_model_path)

        self.model = None
        if model_path:
            self.get_logger().info(f"Cargando modelo YOLO (classify) desde: {model_path}")
            try:
                self.model = YOLO(model_path)
                self.get_logger().info(f"Clases del modelo: {list(self.model.names.values())}")
            except Exception as e:
                self.get_logger().error(f"Error cargando modelo YOLO: {e}")
                self.model = None
        else:
            self.get_logger().warn("No se encontró un modelo YOLO (best.pt). El nodo publicará 'unknown'.")
            self.get_logger().warn("Solución: pasar parámetro 'model_path' o setear env PINCHER_YOLO_MODEL.")

        # Control de frecuencia
        self.last_inference_time = 0.0
        inference_hz = float(self.get_parameter("inference_hz").value)
        inference_hz = inference_hz if inference_hz > 0.0 else 1.0
        self.inference_interval = 1.0 / inference_hz

        # Estado para overlay
        self._last_detected_class = "unknown"
        self._last_confidence = 0.0
        self._last_raw_top1 = None

        self.get_logger().info(f"YOLO Recognition Node Initialized ({inference_hz:.2f}Hz)")
        self.get_logger().info(
            f"Suscrito a: {image_topic} | ROI X: {self.roi_x_min_pct}-{self.roi_x_max_pct}, "
            f"Y: {self.roi_y_min_pct}-{self.roi_y_max_pct} | thr={self.confidence_threshold}"
        )

    def _load_dotenv_if_present(self) -> None:
        env_path = find_dotenv(usecwd=True)
        if env_path:
            load_dotenv(env_path, override=False)

    def _resolve_model_path(self, requested: str) -> str:
        candidates = []
        if requested:
            candidates.append(requested)

        env_path = os.environ.get("PINCHER_YOLO_MODEL", "").strip()
        if env_path:
            candidates.append(env_path)

        try:
            share_dir = Path(get_package_share_directory("pincher_control"))
            candidates.append(str(share_dir / "models" / "best.pt"))
        except Exception:
            pass

        candidates.append(str(Path.cwd() / "runs" / "classify" / "yolo_shapes" / "weights" / "best.pt"))

        for p in candidates:
            pp = Path(p).expanduser()

            # permitir pasar directorio de corrida: .../yolo_shapes3 -> weights/best.pt
            if pp.exists() and pp.is_dir():
                best = pp / "weights" / "best.pt"
                if best.exists() and best.is_file():
                    return str(best)
                best = pp / "best.pt"
                if best.exists() and best.is_file():
                    return str(best)
                last = pp / "weights" / "last.pt"
                if last.exists() and last.is_file():
                    return str(last)
                last = pp / "last.pt"
                if last.exists() and last.is_file():
                    return str(last)

            if pp.exists() and pp.is_file():
                return str(pp)

        self.get_logger().warn("No se encontró modelo en candidatos:")
        for p in candidates:
            self.get_logger().warn(f"  - {p}")
        return ""

    def busy_callback(self, msg: Bool) -> None:
        """Activa/desactiva la inferencia según el estado de la rutina."""
        # True  -> rutina ocupada, no queremos nuevas detecciones
        # False -> rutina libre, reanudamos la detección
        self.vision_enabled = not msg.data

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        height, width, _ = cv_image.shape

        x_min = int(width * self.roi_x_min_pct)
        x_max = int(width * self.roi_x_max_pct)
        y_min = int(height * self.roi_y_min_pct)
        y_max = int(height * self.roi_y_max_pct)

        roi = cv_image[y_min:y_max, x_min:x_max]

        # Si la visión está deshabilitada (rutina en ejecución), solo publicamos debug/ROI.
        if not self.vision_enabled:
            self._draw_and_publish(cv_image, roi, x_min, y_min, x_max, y_max)
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_inference_time >= self.inference_interval:
            self.last_inference_time = current_time
            detected_class, confidence, raw_top1 = self._infer_roi(roi)
            self._last_detected_class = detected_class
            self._last_confidence = confidence
            self._last_raw_top1 = raw_top1

            # Estado continuo
            state_msg = String()
            state_msg.data = (raw_top1 if raw_top1 is not None else "unknown")
            self.figure_state_pub.publish(state_msg)

            # Publicación confirmada
            if detected_class != "unknown" and detected_class != "vacio":
                self.update_buffer(detected_class)
            else:
                self.detection_buffer = []

        self._draw_and_publish(cv_image, roi, x_min, y_min, x_max, y_max)

    def _infer_roi(self, roi):
        detected_class = "unknown"
        confidence = 0.0
        raw_top1 = None

        if self.model is None:
            return detected_class, confidence, raw_top1

        try:
            results = self.model(roi, verbose=False)
            probs = results[0].probs
            if probs is None:
                self.get_logger().warn("Resultado YOLO sin 'probs' (¿modelo no es de clasificación?)")
                return detected_class, confidence, raw_top1

            top1_index = probs.top1
            confidence = float(probs.top1conf.item())
            raw_top1 = results[0].names[top1_index]

            if confidence >= self.confidence_threshold:
                detected_class = raw_top1

            # Rearmar detección cuando el ROI se vacía (independiente del umbral)
            if raw_top1 == "vacio":
                self.vacio_streak += 1
                if self.vacio_streak >= self.vacio_reset_count:
                    if self.last_published_figure:
                        self.get_logger().info("ROI en 'vacio' estable -> rearmando detección.")
                    self.last_published_figure = ""
            else:
                self.vacio_streak = 0

        except Exception as e:
            self.get_logger().error(f"Error ejecutando inferencia YOLO: {e}")

        return detected_class, confidence, raw_top1

    def _draw_and_publish(self, cv_image, roi, x_min, y_min, x_max, y_max):
        detected_class = self._last_detected_class
        confidence = self._last_confidence
        raw_top1 = self._last_raw_top1

        color = (0, 255, 0) if detected_class != "unknown" and detected_class != "vacio" else (0, 0, 255)
        cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), color, 2)

        if raw_top1 is not None and detected_class == "unknown":
            label = f"unknown <=thr | top1={raw_top1} ({confidence:.2f})"
        else:
            label = f"{detected_class} ({confidence:.2f})"

        cv2.putText(
            cv_image,
            label,
            (x_min, y_min - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2,
        )

        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            self.get_logger().error(f"Error publishing debug image: {e}")

        if self.publish_roi:
            try:
                self.roi_pub.publish(self.bridge.cv2_to_imgmsg(roi, "bgr8"))
            except CvBridgeError as e:
                self.get_logger().error(f"Error publishing ROI image: {e}")

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


if __name__ == "__main__":
    main()
