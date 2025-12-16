from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    """
    Bringup de visión para el PhantomX Pincher.

    Más flexible: permite configurar resolución/FPS de la cámara y la frecuencia
    de inferencia de YOLO, y arrancar opcionalmente el clasificador o RViz.
    """

    # Modelo y tópico de imagen vía variables de entorno
    model_path = os.environ.get("PINCHER_YOLO_MODEL", "")
    image_topic_env = os.environ.get("PINCHER_IMAGE_TOPIC", "")

    # Argumentos de launch
    start_camera_arg = DeclareLaunchArgument(
        "start_camera",
        default_value="true",
        description="Si es true, se inicia el nodo de la cámara USB.",
    )
    camera_device_arg = DeclareLaunchArgument(
        "camera_device",
        default_value="/dev/video2",
        description="Dispositivo de vídeo para la cámara USB (p. ej. /dev/video0).",
    )
    image_width_arg = DeclareLaunchArgument(
        "image_width",
        default_value="1280",
        description="Ancho de la imagen de la cámara (píxeles).",
    )
    image_height_arg = DeclareLaunchArgument(
        "image_height",
        default_value="720",
        description="Alto de la imagen de la cámara (píxeles).",
    )
    framerate_arg = DeclareLaunchArgument(
        "framerate",
        default_value="30.0",
        description="Frecuencia de la cámara en Hz (FPS).",
    )
    inference_hz_arg = DeclareLaunchArgument(
        "inference_hz",
        default_value="2.0",
        description="Frecuencia de inferencia para YOLO (Hz).",
    )
    start_clasificador_arg = DeclareLaunchArgument(
        "start_clasificador",
        default_value="false",
        description=(
            "Si es true, se inicia el nodo clasificador aquí. "
            "Normalmente se recomienda lanzarlo desde phantomx_pincher.launch.py."
        ),
    )
    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value="false",
        description="Si es true, se inicia RViz2 desde este launch.",
    )

    # LaunchConfigurations
    start_camera = LaunchConfiguration("start_camera")
    camera_device = LaunchConfiguration("camera_device")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    framerate = LaunchConfiguration("framerate")
    inference_hz = LaunchConfiguration("inference_hz")
    start_clasificador = LaunchConfiguration("start_clasificador")
    start_rviz = LaunchConfiguration("start_rviz")

    # Tópico de imagen que usará YOLO (si está vacío, el nodo decide)
    image_topic = image_topic_env

    # Nodo de la cámara USB
    camera_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        output="screen",
        condition=IfCondition(start_camera),
        parameters=[
            {
                "video_device": camera_device,
                "framerate": framerate,
                "pixel_format": "yuyv",
                "image_width": image_width,
                "image_height": image_height,
                "brightness": -1,
                "contrast": -1,
                "saturation": -1,
                "sharpness": -1,
                "gain": -1,
                "auto_white_balance": True,
                "white_balance": 4000,
                "autoexposure": True,
                "exposure": 100,
                "autofocus": False,
                "focus": -1,
            }
        ],
    )

    # Nodo de reconocimiento de figuras (YOLO)
    yolo_node = Node(
        package="pincher_control",
        executable="yolo_recognition_node",
        name="yolo_recognition_node",
        output="screen",
        parameters=[
            {
                "model_path": model_path,
                "image_topic": image_topic,
                "confidence_threshold": 0.70,
                "inference_hz": inference_hz,
                "publish_roi": True,
            }
        ],
    )

    # Nodo clasificador (Lógica de movimiento) - opcional
    clasificador_node = Node(
        package="pincher_control",
        executable="clasificador_node",
        name="clasificador_node",
        output="screen",
        parameters=[
            {
                "fsm_enabled": True,
                "pause_vision_during_execution": True,
            }
        ],
        condition=IfCondition(start_clasificador),
    )

    # RViz2 (normalmente NO, porque ya lo trae MoveIt/phantomx_pincher.launch.py)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_vision",
        output="screen",
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription(
        [
            start_camera_arg,
            camera_device_arg,
            image_width_arg,
            image_height_arg,
            framerate_arg,
            inference_hz_arg,
            start_clasificador_arg,
            start_rviz_arg,
            camera_node,
            yolo_node,
            clasificador_node,
            rviz_node,
        ]
    )
