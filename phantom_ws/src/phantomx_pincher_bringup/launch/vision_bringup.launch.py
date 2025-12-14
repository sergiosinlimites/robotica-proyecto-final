from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Permite configurar el modelo sin hardcode (export PINCHER_YOLO_MODEL=/path/best.pt)
    model_path = os.environ.get("PINCHER_YOLO_MODEL", "")

    # Permite cambiar el tópico de imagen (USB / Astra / RealSense, etc.)
    # Si está vacío, el nodo escogerá PINCHER_IMAGE_TOPIC o /image_raw automáticamente.
    image_topic = os.environ.get("PINCHER_IMAGE_TOPIC", "")

    start_camera = LaunchConfiguration("start_camera")
    camera_device = LaunchConfiguration("camera_device")
    start_clasificador = LaunchConfiguration("start_clasificador")
    start_rviz = LaunchConfiguration("start_rviz")

    return LaunchDescription([
        # Por defecto este launch NO arranca RViz.
        # Úsalo junto con:
        #   ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py use_real_robot:=true
        # para que RViz/MoveIt vivan en el bringup principal del robot.

        DeclareLaunchArgument(
            "start_camera",
            default_value="true",
            description="Si 'true', arranca usb_cam. Si usas Astra/Orbbec, ponlo en false y lanza el driver aparte.",
        ),
        DeclareLaunchArgument(
            "camera_device",
            default_value="/dev/video2",
            description="Dispositivo de cámara para usb_cam.",
        ),
        DeclareLaunchArgument(
            "start_clasificador",
            default_value="false",
            description="Si 'true', arranca clasificador_node aquí. Recomendado: correrlo en phantomx_pincher.launch.py",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="false",
            description="Si 'true', arranca RViz desde este launch (normalmente NO).",
        ),

        # Nodo de la cámara USB (driver estándar)
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            condition=IfCondition(start_camera),
            parameters=[{
                'video_device': camera_device,
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

        # Nodo clasificador (Lógica de movimiento) - opcional
        Node(
            package='pincher_control',
            executable='clasificador_node',
            name='clasificador_node',
            output='screen',
            condition=IfCondition(start_clasificador),
        ),

        # RViz2 (normalmente NO, porque ya lo trae MoveIt/phantomx_pincher.launch.py)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_vision',
            output='screen',
            condition=IfCondition(start_rviz),
        ),
    ])
