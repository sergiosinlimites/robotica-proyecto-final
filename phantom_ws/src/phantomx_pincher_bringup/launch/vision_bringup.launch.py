from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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
        ),

        # Nodo clasificador (Lógica de movimiento)
        Node(
            package='pincher_control',
            executable='clasificador_node',
            name='clasificador_node',
            output='screen',
        ),
    ])
