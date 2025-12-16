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
    def __init__(self) -> None:
        super().__init__("yolo_recognition_node")

        self._load_dotenv_if_present()

        # ------------------------
        # Parámetros ROS
        # ------------------------
        # Si queda vacío, tomamos PINCHER_IMAGE_TOPIC o /image_raw
        self.declare_parameter("image_topic", "")
        self.declare_parameter("model_path", "")
        self.declare_parameter("confidence_threshold", 0.7)
        self.declare_parameter("inference_hz", 2.0)
        self.declare_parameter("publish_roi", True)

        self.declare_parameter("roi_x_min_pct", 0.45)
        self.declare_parameter("roi_x_max_pct", 0.60)
        self.declare_parameter("roi_y_min_pct", 0.62)
        self.declare_parameter("roi_y_max_pct", 0.77)

        # Estabilización de detección
        self.declare_parameter("buffer_size", 10)
        self.declare_parameter("vacio_reset_count", 3)

        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value.strip()
        )
        if not image_topic:
            image_topic = os.environ.get("PINCHER_IMAGE_TOPIC", "/image_raw").strip() or "/image_raw"

        # Suscripción a la cámara
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )

        # Publicadores de salida
        # - /figure_type: sólo se publica cuando hay detección estable (NO vacio/unknown)
        # - /figure_state: estado continuo (incluye vacio/unknown) para gating del ejecutor de rutinas
        self.figure_pub = self.create_publisher(String, "/figure_type", 10)
        self.figure_state_pub = self.create_publisher(String, "/figure_state", 10)
        self.debug_pub = self.create_publisher(Image, "/camera/debug", 10)
        self.roi_pub = self.create_publisher(Image, "/camera/roi", 10)

        # Permite pausar la detección mientras una rutina está en ejecución
        # (clasificador_node publica /routine_busy=True durante la secuencia).
        self.vision_enabled = True
        self._was_busy = False
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
        self.buffer_size = int(self.get_parameter("buffer_size").value)
        self.last_published_figure = ""
        self.vacio_streak = 0
        self.vacio_reset_count = int(self.get_parameter("vacio_reset_count").value)

        # ------------------------
        # Cargar Modelo YOLO (clasificación)
        # ------------------------
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.publish_roi = bool(self.get_parameter("publish_roi").value)

        requested_model_path = (
            self.get_parameter("model_path").get_parameter_value().string_value.strip()
        )
        model_path = self._resolve_model_path(requested_model_path)

        self.model = None
        if model_path:
            self.get_logger().info(f"Cargando modelo YOLO (classify) desde: {model_path}")
            try:
                self.model = YOLO(model_path)
                self.get_logger().info(
                    f"Clases del modelo: {list(self.model.names.values())}"
                )
            except Exception as e:
                self.get_logger().error(f"Error cargando modelo YOLO: {e}")
                self.model = None
        else:
            self.get_logger().warn(
                "No se encontró un modelo YOLO (best.pt). El nodo publicará 'unknown'."
            )
            self.get_logger().warn(
                "Solución: pasar parámetro 'model_path' o setear env PINCHER_YOLO_MODEL."
            )

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

    # ------------------------------------------------------------------
    # Callbacks de control y suscripción
    # ------------------------------------------------------------------
    def busy_callback(self, msg: Bool) -> None:
        """Activa/desactiva la inferencia según el estado de la rutina.

        Cuando la rutina pasa de ocupada→libre, se rearma la lógica de
        publicación para permitir nuevas detecciones (incluyendo la misma clase).
        """
        was_busy = self._was_busy
        is_busy = bool(msg.data)

        self.vision_enabled = not is_busy
        self._was_busy = is_busy

        # Si veníamos de estar ocupados y ahora ya no, limpiamos el buffer y el
        # "último" tipo publicado, para que una nueva figura (incluso del mismo
        # tipo) vuelva a disparar una rutina sin necesidad de ver 'vacio'.
        if was_busy and not is_busy:
            self.get_logger().info("Rutina finalizada → rearmando detección YOLO.")
            self.detection_buffer = []
            self.last_published_figure = ""
            self.vacio_streak = 0

    def image_callback(self, msg: Image) -> None:
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

        # Si la visión está deshabilitada (rutina en ejecución), sólo publicamos debug/ROI.
        if not self.vision_enabled:
            self._draw_and_publish(cv_image, roi, x_min, y_min, x_max, y_max)
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_inference_time >= self.inference_interval:
            self.last_inference_time = now

            detected_class, confidence, raw_top1 = self._infer_roi(roi)
            self._last_detected_class = detected_class
            self._last_confidence = confidence
            self._last_raw_top1 = raw_top1

            # Estado continuo
            state_msg = String()
            state_msg.data = raw_top1 if raw_top1 is not None else "unknown"
            self.figure_state_pub.publish(state_msg)

            # Publicación confirmada (usa buffer para estabilizar)
            if detected_class not in ("unknown", "vacio"):
                self.update_buffer(detected_class)
            else:
                # Si la detección no es estable o es vacío, vaciamos el buffer
                self.detection_buffer = []

        # Publicar imagen de depuración y ROI (última detección conocida)
        self._draw_and_publish(cv_image, roi, x_min, y_min, x_max, y_max)

    # ------------------------------------------------------------------
    # Utilidades internas
    # ------------------------------------------------------------------
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

        candidates.append(
            str(Path.cwd() / "runs" / "classify" / "yolo_shapes" / "weights" / "best.pt")
        )

        for p in candidates:
            pp = Path(p).expanduser()

            # permitir pasar directorio de corrida: .../yolo_shapesX -> weights/best.pt
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
                self.get_logger().warn(
                    "Resultado YOLO sin 'probs' (¿modelo no es de clasificación?)"
                )
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
                        self.get_logger().info(
                            "ROI en 'vacio' estable -> rearmando detección."
                        )
                    self.last_published_figure = ""
            else:
                self.vacio_streak = 0

        except Exception as e:
            self.get_logger().error(f"Error ejecutando inferencia YOLO: {e}")

        return detected_class, confidence, raw_top1

    def _draw_and_publish(self, cv_image, roi, x_min, y_min, x_max, y_max) -> None:
        detected_class = self._last_detected_class
        confidence = self._last_confidence
        raw_top1 = self._last_raw_top1

        color = (
            (0, 255, 0)
            if detected_class not in ("unknown", "vacio")
            else (0, 0, 255)
        )
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

    def update_buffer(self, shape: str) -> None:
        """Acumula detecciones y publica /figure_type sólo cuando son estables.

        Se requiere que los últimos `buffer_size` elementos del buffer coincidan
        con la misma clase para considerarla confirmada. Además, para evitar
        disparos repetidos, sólo se publica cuando la nueva clase es distinta
        de `last_published_figure`.
        """
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


def main(args=None) -> None:
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
