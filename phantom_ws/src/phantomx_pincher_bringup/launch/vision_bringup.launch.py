from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Permite configurar el modelo sin hardcode (export PINCHER_YOLO_MODEL=/path/best.pt)
    model_path = os.environ.get("PINCHER_YOLO_MODEL", "")

    # Permite cambiar el tópico de imagen (USB / Astra / RealSense, etc.)
    # Si está vacío, el nodo escogerá PINCHER_IMAGE_TOPIC o /image_raw automáticamente.
    image_topic = os.environ.get("PINCHER_IMAGE_TOPIC", "")

    return LaunchDescription([
        # Nodo de la cámara USB (driver estándar)
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video2',
                'framerate': 30.0,
                'pixel_format': 'yuyv',
                'image_width': 640,
                'image_height': 480,
                'brightness': -1,
                'contrast': -1,
                'saturation': -1,
                'sharpness': -1,
                'gain': -1,
                'auto_white_balance': True,
                'white_balance': 4000,
                'autoexposure': True,
                'exposure': 100,
                'autofocus': False,
                'focus': -1,
            }]
        ),

        # Nodo de reconocimiento de figuras
        Node(
            package='pincher_control',
            executable='yolo_recognition_node',
            name='yolo_recognition_node',
            output='screen',
            parameters=[{
                'model_path': model_path,
                'image_topic': image_topic,
                'confidence_threshold': 0.70,
                'inference_hz': 2.0,
                'publish_roi': True,
            }],
        ),

        # Nodo clasificador (Lógica de movimiento)
        Node(
            package='pincher_control',
            executable='clasificador_node',
            name='clasificador_node',
            output='screen',
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # arguments=['-d', '/path/to/config.rviz'] # Opcional: cargar config guardada
        ),
    ])
