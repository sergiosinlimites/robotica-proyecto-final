import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
import time
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import subprocess
import os
import math
import numpy as np

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

try:
    from phantomx_pincher_interfaces.msg import PoseCommand
except Exception:
    PoseCommand = None  # type: ignore

# Desactiva siempre el uso de PoseCommand para evitar errores de typesupport C.
# La pestaña "Espacio de la Tarea" seguirá mostrando un mensaje de que no está disponible.
HAS_POSE_COMMAND = False

MAX_ANGLE_DEG = 150.0
MIN_ANGLE_DEG = -150.0

# ============================================================
#  CONFIGURACIÓN: ¿QUÉ MOTORES ESTÁS USANDO?
# ============================================================
USE_XL430 = False

# ------------------------------------------------------------
# Direcciones de registro y parámetros según el tipo de motor
# ------------------------------------------------------------
if USE_XL430:
    PROTOCOL_VERSION = 2.0
    ADDR_TORQUE_ENABLE    = 64
    ADDR_GOAL_POSITION    = 116
    ADDR_MOVING_SPEED     = 112  # Profile Velocity
    ADDR_TORQUE_LIMIT     = 38
    ADDR_PRESENT_POSITION = 132
    DEFAULT_GOAL = 2048
    MAX_SPEED = 1023  # Velocidad máxima para XL430
else:
    PROTOCOL_VERSION = 1.0
    ADDR_TORQUE_ENABLE    = 24
    ADDR_GOAL_POSITION    = 30
    ADDR_MOVING_SPEED     = 32
    ADDR_TORQUE_LIMIT     = 34
    ADDR_PRESENT_POSITION = 36
    DEFAULT_GOAL = 512
    MAX_SPEED = 1023  # Velocidad máxima para otros modelos

# ============================================================
#  FUNCIONES AUXILIARES
# ============================================================

def write_goal_position(packet, port, dxl_id, position):
    if USE_XL430:
        return packet.write4ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, int(position))
    else:
        return packet.write2ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, int(position))

def write_moving_speed(packet, port, dxl_id, speed):
    if USE_XL430:
        return packet.write4ByteTxRx(port, dxl_id, ADDR_MOVING_SPEED, int(speed))
    else:
        return packet.write2ByteTxRx(port, dxl_id, ADDR_MOVING_SPEED, int(speed))

def read_present_position(packet, port, dxl_id):
    if USE_XL430:
        return packet.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
    else:
        return packet.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)

# ============================================================
#  NODO ROS2 CON PUBLICACIÓN PARA RViz
# ============================================================

class PincherController(Node):
    def __init__(self):
        super().__init__('pincher_controller')

        # Parámetros
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('dxl_ids', [1, 2, 3, 4, 5])
        self.declare_parameter('goal_positions', [DEFAULT_GOAL] * 5)
        self.declare_parameter('moving_speed', 100)
        self.declare_parameter('torque_limit', 800)
        
        # Obtener parámetros
        port_name = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.dxl_ids = self.get_parameter('dxl_ids').value
        goal_positions = self.get_parameter('goal_positions').value
        moving_speed = int(self.get_parameter('moving_speed').value)
        torque_limit = int(self.get_parameter('torque_limit').value)

        # Mapeo de IDs de motor a nombres de articulaciones del URDF
        self.joint_names = [
            'phantomx_pincher_arm_shoulder_pan_joint',
            'phantomx_pincher_arm_shoulder_lift_joint',
            'phantomx_pincher_arm_elbow_flex_joint',
            'phantomx_pincher_arm_wrist_flex_joint',
            'phantomx_pincher_gripper_finger1_joint',  # usamos el dedo 1 como gripper
        ]

        self.joint_sign = {
            1:  1,
            2: -1,
            3: -1,
            4: -1,
            5:  1,
        }

        # Inicializar comunicación
        self.port = PortHandler(port_name)
        if not self.port.openPort():
            self.get_logger().error(f'No se pudo abrir el puerto {port_name}')
            rclpy.shutdown()
            return

        if not self.port.setBaudRate(baudrate):
            self.get_logger().error(f'No se pudo configurar baudrate={baudrate}')
            self.port.closePort()
            rclpy.shutdown()
            return

        self.packet = PacketHandler(PROTOCOL_VERSION)
        
        # Posiciones actuales de las articulaciones (en radianes / metros)
        # Mantén el tamaño sincronizado con el número de motores (dxl_ids)
        self.current_joint_positions = [0.0] * 5  # Para 5 articulaciones / ejes

        # Estado de emergencia
        self.emergency_stop_activated = False
        
        # Configuración inicial de los motores
        self.initialize_motors(goal_positions, moving_speed, torque_limit)
        
        # Publicador para Joint States (para RViz)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Timer para publicar joint states
        self.joint_state_timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz

        # Longitudes de eslabones para cinemática directa (m), según DH estándar proporcionado
        self.L1 = 0.045
        self.L2 = 0.107
        self.L3 = 0.107
        self.L4 = 0.109

        # Pose cartesiana actual del TCP (x, y, z en metros) y orientación RPY (rad)
        self.current_tcp_xyz = (0.0, 0.0, 0.0)
        self.current_tcp_rpy = (0.0, 0.0, 0.0)
        
        # Offset en Z aplicado sobre el primer eslabón (L1) para alinear
        # la cinemática DH con la referencia que usas en RViz (punta de la garra).
        # Solo afecta a la visualización del TCP, no al control del robot.
        self.tcp_z_offset = -0.02  # metros

        # Publicador de la pose del TCP (calculada con cinemática directa DH)
        self.tcp_pose_pub = self.create_publisher(PoseStamped, '/tcp_pose', 10)
        # Publicador de marcador de texto con XYZ del TCP para RViz
        self.tcp_marker_pub = self.create_publisher(Marker, '/tcp_pose_marker', 10)
        self.tcp_pose_timer = self.create_timer(0.1, self.update_tcp_pose)  # 10 Hz

        # Publicador de comandos en espacio de la tarea (X, Y, Z, R, P, Y) hacia MoveIt
        if HAS_POSE_COMMAND:
            self.pose_command_pub = self.create_publisher(PoseCommand, 'pose_command', 10)
        else:
            self.pose_command_pub = None
            self.get_logger().warn(
                "PoseCommand no disponible (phantomx_pincher_interfaces sin soporte C); "
                "la pestaña 'Espacio de la Tarea' solo mostrará un aviso."
            )

    def initialize_motors(self, goal_positions, moving_speed, torque_limit):
        """Configuración inicial de todos los motores"""
        for dxl_id, goal in zip(self.dxl_ids, goal_positions):
            try:
                # Habilitar torque
                result, error = self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 1)
                if result != 0:
                    self.get_logger().error(f'Error habilitando torque en motor {dxl_id}: {error}')
                    continue
                
                # Configurar velocidad
                self.update_speed_single_motor(dxl_id, moving_speed)
                
                # Mover a posición inicial
                write_goal_position(self.packet, self.port, dxl_id, goal)
                
                # Actualizar posición de articulación para RViz
                joint_index = self.dxl_ids.index(dxl_id)
                angle = self.dxl_to_radians(goal)
                angle *= self.joint_sign.get(dxl_id, 1)
                self.current_joint_positions[joint_index] = angle
                
                self.get_logger().info(f'Motor {dxl_id} configurado correctamente')
                
            except Exception as e:
                self.get_logger().error(f'Error configurando motor {dxl_id}: {str(e)}')

    def dxl_to_radians(self, dxl_value):
        """Convierte valor Dynamixel a radianes (-2.618 a 2.618 aprox.).

        - Para AX-12 / AX compatibles (USE_XL430 = False):
          rango 0‑1023, centro en ~512 ⇒ 0 rad en 512.
        - Para XL430 (USE_XL430 = True):
          rango 0‑4095, centro en ~2048 ⇒ 0 rad en 2048.

        Esta función SOLO afecta a lo que ve RViz (joint_states),
        no a los comandos que se envían al motor físico.
        """
        if USE_XL430:
            center = 2048.0
            scale = 2.618 / 2048.0
        else:
            center = 512.0
            scale = 2.618 / 512.0

        return (dxl_value - center) * scale

    def radians_to_dxl(self, radians):
        """Convierte radianes a valor Dynamixel."""
        if USE_XL430:
            center = 2048.0
            inv_scale = 2048.0 / 2.618
        else:
            center = 512.0
            inv_scale = 512.0 / 2.618

        return int(radians * inv_scale + center)

    def dxl_to_degrees(self, dxl_value, motor_id):
        """Convierte un valor Dynamixel a grados, respetando el signo del joint."""
        angle_rad = self.dxl_to_radians(dxl_value)
        angle_rad *= self.joint_sign.get(motor_id, 1)
        return angle_rad * 180.0 / math.pi

    def degrees_to_dxl(self, degrees, motor_id):
        """Convierte grados (respecto a HOME = 0°) a valor Dynamixel.

        IMPORTANTE: 0° corresponde exactamente a DEFAULT_GOAL, por lo que
        el HOME físico no se mueve al cambiar de ticks a grados.
        """
        angle_rad = degrees * math.pi / 180.0
        angle_rad *= self.joint_sign.get(motor_id, 1)
        return self.radians_to_dxl(angle_rad)

    def publish_joint_states(self):
        """Publica el estado de las articulaciones para RViz"""
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = "base_link"
        
        joint_state.name = self.joint_names
        joint_state.position = self.current_joint_positions
        
        self.joint_state_pub.publish(joint_state)

    # ============================================================
    #  CINEMÁTICA DIRECTA POR DH (SIN TF)
    # ============================================================

    def _dh_standard_np(self, a, alpha, d, theta):
        """Devuelve la matriz A_i(theta_i) de DH estándar como numpy.array.

        A_i(q_i) = trotz(theta_i) * transl(0,0,d_i) * transl(a_i,0,0) * trotx(alpha_i)
        """
        ca = math.cos(alpha)
        sa = math.sin(alpha)
        ct = math.cos(theta)
        st = math.sin(theta)

        return np.array(
            [
                [ct, -st * ca,  st * sa, a * ct],
                [st,  ct * ca, -ct * sa, a * st],
                [0.0,     sa,      ca,       d],
                [0.0,    0.0,     0.0,     1.0],
            ],
            dtype=float,
        )

    def _matrix_to_rpy(self, R):
        """Convierte una matriz de rotación 3x3 a ángulos roll, pitch, yaw (rad).

        Convención: Rotación Z-Y-X (yaw-pitch-roll), consistente con RPY estándar.
        """
        # Evitar problemas numéricos
        r20 = max(min(R[2, 0], 1.0), -1.0)
        pitch = math.asin(-r20)

        if abs(r20) < 0.9999:
            roll = math.atan2(R[2, 1], R[2, 2])
            yaw = math.atan2(R[1, 0], R[0, 0])
        else:
            # Singularidad (gimbal lock)
            roll = 0.0
            yaw = math.atan2(-R[0, 1], R[1, 1])

        return roll, pitch, yaw

    def _rpy_to_quaternion(self, roll, pitch, yaw):
        """Convierte RPY (rad) a un cuaternión (x, y, z, w)."""
        cr = math.cos(roll / 2.0)
        sr = math.sin(roll / 2.0)
        cp = math.cos(pitch / 2.0)
        sp = math.sin(pitch / 2.0)
        cy = math.cos(yaw / 2.0)
        sy = math.sin(yaw / 2.0)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    def compute_tcp_fk(self):
        """Calcula X,Y,Z y R,P,Y del TCP usando DH estándar y los 4 primeros joints.

        Usa las longitudes L1..L4 proporcionadas por ti:
            L1 = 0.045
            L2 = 0.107
            L3 = 0.107
            L4 = 0.109

        Y los ángulos articulares actuales (radianes) de:
            q1 = base
            q2 = shoulder
            q3 = elbow
            q4 = wrist
        """
        if len(self.current_joint_positions) < 4:
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

        # Ángulos articulares actuales (radianes) en el convenio de ROS/URDF.
        # Aplicamos offsets para que la configuración HOME del robot (q=0)
        # coincida con la configuración de referencia que usaste en Matlab:
        #   q1_dh = -pi/2
        #   q2_dh = -pi/2
        #   q3_dh = 0
        #   q4_dh = 0
        q1_ros = self.current_joint_positions[0]
        q2_ros = self.current_joint_positions[1]
        q3_ros = self.current_joint_positions[2]
        q4_ros = self.current_joint_positions[3]

        q1 = q1_ros - math.pi / 2.0
        q2 = q2_ros - math.pi / 2.0
        q3 = q3_ros
        q4 = q4_ros

        # Ajuste solo sobre el primer eslabón (L1) para alinear en Z
        L1 = self.L1 + self.tcp_z_offset
        L2 = self.L2
        L3 = self.L3
        L4 = self.L4

        # Matrices DH estándar según el código de Matlab que enviaste
        A1 = self._dh_standard_np(0.0, -math.pi / 2.0, L1, q1)
        A2 = self._dh_standard_np(L2, 0.0, 0.0, q2)
        A3 = self._dh_standard_np(L3, 0.0, 0.0, q3)
        A4 = self._dh_standard_np(L4, 0.0, 0.0, q4)

        H = A1 @ A2 @ A3 @ A4

        # Separar rotación y traslación en el marco DH "puro"
        R_dh = H[0:3, 0:3]
        x_dh = float(H[0, 3])
        y_dh = float(H[1, 3])
        z = float(H[2, 3])

        # Ajuste de ejes para que coincida con el sistema de la base del robot:
        #   - Rotación de +90° alrededor de Z:
        #       x_base = -y_dh
        #       y_base =  x_dh
        Rz90 = np.array(
            [
                [0.0, -1.0, 0.0],
                [1.0,  0.0, 0.0],
                [0.0,  0.0, 1.0],
            ],
            dtype=float,
        )
        R_base = Rz90 @ R_dh

        x_base = -y_dh
        y_base = x_dh

        roll, pitch, yaw = self._matrix_to_rpy(R_base)

        return x_base, y_base, z, roll, pitch, yaw

    def update_tcp_pose(self):
        """Actualiza la pose cartesiana del TCP usando cinemática directa DH estándar."""
        x, y, z, roll, pitch, yaw = self.compute_tcp_fk()

        # Actualizar estado interno
        self.current_tcp_xyz = (x, y, z)
        self.current_tcp_rpy = (roll, pitch, yaw)

        # Publicar PoseStamped para RViz / debug
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()
        # Frame base aproximado del modelo DH (equivalente a la base del brazo)
        pose.header.frame_id = "phantomx_pincher_arm_base_link"

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        # Orientación completa a partir de roll, pitch, yaw
        qx, qy, qz, qw = self._rpy_to_quaternion(roll, pitch, yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.tcp_pose_pub.publish(pose)

        # Publicar marcador de texto con XYZ en RViz
        marker = Marker()
        marker.header = pose.header
        marker.ns = "tcp_pose"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.z = 0.025  # tamaño del texto (altura de letras)
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        # Mostrar también RPY en grados
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        marker.text = (
            f"X={x:.3f}\nY={y:.3f}\nZ={z:.3f}\n"
            f"R={roll_deg:.1f}°\nP={pitch_deg:.1f}°\nY={yaw_deg:.1f}°"
        )

        self.tcp_marker_pub.publish(marker)


    def move_motor(self, motor_id, position):
        """Mueve un motor a la posición especificada solo si no hay emergencia y velocidad > 0"""
        if self.emergency_stop_activated:
            self.get_logger().warning(f'No se puede mover motor {motor_id}: Parada de emergencia activada')
            return
            
        try:
            result, error = write_goal_position(self.packet, self.port, motor_id, position)
            if result == 0:
                self.get_logger().info(f'[Motor {motor_id}] Moviendo a {position}')
                
                # Actualizar posición de articulación para RViz
                joint_index = self.dxl_ids.index(motor_id)
                angle = self.dxl_to_radians(position)
                angle *= self.joint_sign.get(motor_id, 1)
                self.current_joint_positions[joint_index] = angle
                
            else:
                self.get_logger().error(f'Error moviendo motor {motor_id}: {error}')
        except Exception as e:
            self.get_logger().error(f'Excepción moviendo motor {motor_id}: {str(e)}')

    def update_speed_single_motor(self, motor_id, speed):
        """Actualiza la velocidad de un motor individual"""
        try:
            result, error = write_moving_speed(self.packet, self.port, motor_id, speed)
            return result == 0
        except Exception as e:
            self.get_logger().error(f'Error actualizando velocidad motor {motor_id}: {str(e)}')
            return False

    def update_speed(self, speed):
        """Actualiza la velocidad de movimiento en todos los motores"""
        if self.emergency_stop_activated:
            self.get_logger().warning('No se puede actualizar velocidad: Parada de emergencia activada')
            return
            
        success_count = 0
        for motor_id in self.dxl_ids:
            if self.update_speed_single_motor(motor_id, speed):
                success_count += 1
        
        if success_count == len(self.dxl_ids):
            self.get_logger().info(f'Velocidad actualizada a {speed} en todos los motores')
        else:
            self.get_logger().warning(f'Velocidad actualizada a {speed} en {success_count}/{len(self.dxl_ids)} motores')

    def home_all_motors(self):
        """Mueve todos los motores a la posición home (DEFAULT_GOAL)"""
        if self.emergency_stop_activated:
            # Reactivar torque si hay emergencia
            self.reactivate_torque()
            
        for motor_id in self.dxl_ids:
            self.move_motor(motor_id, DEFAULT_GOAL)
        self.get_logger().info('Todos los motores movidos a posición HOME')

    def emergency_stop(self):
        """Parada de emergencia - desactiva el torque de todos los motores"""
        self.emergency_stop_activated = True
        for dxl_id in self.dxl_ids:
            try:
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 0)
                self.get_logger().warning(f'Torque desactivado en motor {dxl_id} (EMERGENCY STOP)')
            except Exception as e:
                self.get_logger().error(f'Error en parada de emergencia motor {dxl_id}: {str(e)}')

    def reactivate_torque(self):
        """Reactivar el torque después de una parada de emergencia"""
        self.emergency_stop_activated = False
        for dxl_id in self.dxl_ids:
            try:
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 1)
                self.get_logger().info(f'Torque reactivado en motor {dxl_id}')
            except Exception as e:
                self.get_logger().error(f'Error reactivando torque en motor {dxl_id}: {str(e)}')

    def close(self):
        """Apaga el torque y cierra el puerto"""
        for dxl_id in self.dxl_ids:
            try:
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 0)
            except:
                pass
        self.port.closePort()

# ============================================================
#  INTERFAZ GRÁFICA CON PESTAÑAS (INCLUYENDO RViz)
# ============================================================

class PincherGUI:
    def __init__(self, controller):
        self.controller = controller
        self.window = tk.Tk()
        self.window.title("Control Pincher - Interfaz Completa")
        self.window.protocol("WM_DELETE_WINDOW", self.on_close)

        # Variables para control de actualización
        self.last_motor_update = {motor_id: 0 for motor_id in controller.dxl_ids}
        self.last_speed_update = 0
        self.update_interval = 0.05  # 50ms entre actualizaciones

        # Proceso de RViz
        self.rviz_process = None

        # Estado de ejecución de secuencias de pose (solo una a la vez)
        self.pose_sequence_running = False

        # Crear notebook (pestañas)
        self.notebook = ttk.Notebook(self.window)
        self.notebook.pack(fill='both', expand=True, padx=10, pady=10)

        # Crear frames para cada pestaña
        self.tab1 = ttk.Frame(self.notebook)
        self.tab2 = ttk.Frame(self.notebook)
        self.tab3 = ttk.Frame(self.notebook)  # Visualización RViz
        self.tab4 = ttk.Frame(self.notebook)  # Control por pose articular
        self.tab5 = ttk.Frame(self.notebook)  # Pestaña "Acerca de"
        self.tab6 = ttk.Frame(self.notebook)  # Control en espacio de la tarea

        self.notebook.add(self.tab1, text='Control por Sliders')
        self.notebook.add(self.tab2, text='Control por Valores')
        self.notebook.add(self.tab3, text='Visualización RViz')
        self.notebook.add(self.tab4, text='Control por Pose')
        self.notebook.add(self.tab6, text='Espacio de la Tarea')
        self.notebook.add(self.tab5, text='Acerca de')

        # Configurar las pestañas
        self.setup_tab1()
        self.setup_tab2()
        self.setup_tab3()
        self.setup_tab4()
        self.setup_tab6()
        self.setup_tab5()

        # Barra de botones comunes en la parte inferior
        self.setup_common_buttons()

        # Barra global con XYZ y RPY del TCP (visible en todas las pestañas)
        self.setup_tcp_status_bar()

        # Mostrar/ocultar barra TCP según pestaña (no se muestra en "Acerca de")
        self.notebook.bind("<<NotebookTabChanged>>", self.on_tab_changed)

        # Iniciar actualización periódica de articulaciones y pose TCP
        self.update_joints_timer()

    def setup_tab1(self):
        """Configura la pestaña 1: Control por Sliders en Tiempo Real"""
        # Título
        title_label = tk.Label(self.tab1, text="Control por Sliders - TIEMPO REAL", 
                              font=("Arial", 14, "bold"))
        title_label.pack(pady=10)
        
        # Frame para los sliders de motores
        motors_frame = tk.Frame(self.tab1)
        motors_frame.pack(fill='x', padx=20, pady=10)
        
        self.sliders = {}
        self.labels = {}
        
        # Sliders para cada motor (en grados, respecto a HOME = 0°)
        for i, motor_id in enumerate(self.controller.dxl_ids):
            motor_frame = tk.Frame(motors_frame)
            motor_frame.pack(fill='x', pady=5)
            
            # Label del motor
            motor_label = tk.Label(motor_frame, text=f'Motor {motor_id}', 
                                  font=("Arial", 10, "bold"), width=8)
            motor_label.pack(side='left', padx=5)
            
            # Slider en grados
            slider = tk.Scale(
                motor_frame,
                from_=MIN_ANGLE_DEG,
                to=MAX_ANGLE_DEG,
                orient=tk.HORIZONTAL,
                length=400,
                showvalue=True,
                resolution=1,
                command=lambda value, mid=motor_id: self.on_motor_slider_change(mid)
            )
            slider.set(0)  # HOME = 0°
            slider.pack(side='left', fill='x', expand=True, padx=5)
            
            # Etiqueta de posición
            label = tk.Label(
                motor_frame,
                text=f'Pos: 0.0°',
                font=("Arial", 9),
                width=12
            )
            label.pack(side='right', padx=5)
            
            self.sliders[motor_id] = slider
            self.labels[motor_id] = label
        
        # Separador
        ttk.Separator(self.tab1, orient='horizontal').pack(fill='x', padx=20, pady=10)
        
        # Frame para control de velocidad
        speed_frame = tk.Frame(self.tab1)
        speed_frame.pack(fill='x', padx=20, pady=10)
        
        speed_label = tk.Label(speed_frame, text="Velocidad de Movimiento:", 
                              font=("Arial", 10, "bold"))
        speed_label.pack(anchor='w')
        
        speed_control_frame = tk.Frame(speed_frame)
        speed_control_frame.pack(fill='x', pady=5)
        
        self.speed_slider_tab1 = tk.Scale(speed_control_frame, from_=0, to=MAX_SPEED, 
                                         orient=tk.HORIZONTAL, length=400,
                                         showvalue=True, resolution=1,
                                         command=self.on_speed_slider_change)
        self.speed_slider_tab1.set(100)
        self.speed_slider_tab1.pack(side='left', fill='x', expand=True)
        
        self.speed_value_label_tab1 = tk.Label(speed_control_frame, text="100", 
                                              font=("Arial", 10, "bold"), width=5)
        self.speed_value_label_tab1.pack(side='right', padx=10)
        
        # Nota sobre velocidad 0
        speed_note = tk.Label(speed_frame, text="Nota: Velocidad 0 = No movimiento", 
                             font=("Arial", 8), fg="gray")
        speed_note.pack(anchor='w')

    def setup_tab2(self):
        """Configura la pestaña 2: Control por Valores"""
        # Título
        title_label = tk.Label(self.tab2, text="Control por Valores Manuales", 
                              font=("Arial", 14, "bold"))
        title_label.pack(pady=10)
        
        # Frame para control de velocidad
        speed_frame = tk.Frame(self.tab2)
        speed_frame.pack(fill='x', padx=20, pady=10)
        
        speed_label = tk.Label(speed_frame, text="Velocidad de Movimiento:", 
                              font=("Arial", 10, "bold"))
        speed_label.pack(anchor='w')
        
        speed_control_frame = tk.Frame(speed_frame)
        speed_control_frame.pack(fill='x', pady=5)
        
        self.speed_slider_tab2 = tk.Scale(speed_control_frame, from_=0, to=MAX_SPEED, 
                                         orient=tk.HORIZONTAL, length=400,
                                         showvalue=True, resolution=1,
                                         command=self.on_speed_slider_change)
        self.speed_slider_tab2.set(100)
        self.speed_slider_tab2.pack(side='left', fill='x', expand=True)
        
        self.speed_value_label_tab2 = tk.Label(speed_control_frame, text="100", 
                                              font=("Arial", 10, "bold"), width=5)
        self.speed_value_label_tab2.pack(side='right', padx=10)
        
        # Nota sobre velocidad 0
        speed_note = tk.Label(speed_frame, text="Nota: Velocidad 0 = No movimiento", 
                             font=("Arial", 8), fg="gray")
        speed_note.pack(anchor='w')
        
        # Separador
        ttk.Separator(self.tab2, orient='horizontal').pack(fill='x', padx=20, pady=10)
        
        # Frame para control individual de motores
        motors_frame = tk.Frame(self.tab2)
        motors_frame.pack(fill='both', expand=True, padx=20, pady=10)
        
        self.entries = {}
        self.entry_labels = {}
        
        # Crear filas para cada motor (entrada en grados)
        for i, motor_id in enumerate(self.controller.dxl_ids):
            motor_frame = tk.Frame(motors_frame)
            motor_frame.pack(fill='x', pady=8)
            
            # Label del motor
            motor_label = tk.Label(motor_frame, text=f'Motor {motor_id}', 
                                  font=("Arial", 10, "bold"), width=8)
            motor_label.pack(side='left', padx=5)
            
            # Entry para valor en grados
            entry_label = tk.Label(
                motor_frame,
                text=f"Ángulo ({int(MIN_ANGLE_DEG)} a {int(MAX_ANGLE_DEG)} °):",
                font=("Arial", 9)
            )
            entry_label.pack(side='left', padx=5)
            
            entry = tk.Entry(motor_frame, width=8, font=("Arial", 10))
            entry.insert(0, "0")  # HOME = 0°
            entry.pack(side='left', padx=5)
            
            # Botón para mover motor individual
            move_btn = tk.Button(motor_frame, text="Mover Motor", 
                                font=("Arial", 9),
                                command=lambda mid=motor_id: self.move_single_motor_from_entry(mid))
            move_btn.pack(side='left', padx=5)
            
            # Etiqueta de estado
            status_label = tk.Label(motor_frame, text="Listo", 
                                   font=("Arial", 9), fg="green", width=10)
            status_label.pack(side='right', padx=5)
            
            self.entries[motor_id] = entry
            self.entry_labels[motor_id] = status_label
            
            # Bind Enter key para mover automáticamente
            entry.bind('<Return>', lambda event, mid=motor_id: self.move_single_motor_from_entry(mid))
        
        # Botón para mover todos los motores
        move_all_frame = tk.Frame(motors_frame)
        move_all_frame.pack(fill='x', pady=15)
        
        move_all_btn = tk.Button(move_all_frame, text="MOVER TODOS LOS MOTORES", 
                                font=("Arial", 10, "bold"), bg="#4CAF50", fg="white",
                                command=self.move_all_motors_from_entries)
        move_all_btn.pack(pady=10)

    def setup_tab3(self):
        """Configura la pestaña 3: Visualización en RViz"""
        # Título
        title_label = tk.Label(self.tab3, text="Visualización en RViz - PhantomX Pincher X100", 
                              font=("Arial", 14, "bold"))
        title_label.pack(pady=10)
        
        # Frame de información
        info_frame = tk.Frame(self.tab3)
        info_frame.pack(fill='x', padx=20, pady=10)
        
        info_text = """
Esta pestaña permite visualizar el robot PhantomX Pincher X100 en RViz.

Características:
• Modelo 3D del PhantomX Pincher X100
• Sincronización en tiempo real con el robot físico
• Visualización de todas las articulaciones
• Feedback visual de los movimientos
        """
        info_label = tk.Label(info_frame, text=info_text, justify=tk.LEFT, 
                             font=("Arial", 10), bg="#f0f0f0", relief="solid", padx=10, pady=10)
        info_label.pack(fill='x')
        
        # Frame de controles RViz
        controls_frame = tk.Frame(self.tab3)
        controls_frame.pack(fill='x', padx=20, pady=20)
        
        # Botón para lanzar RViz
        self.rviz_btn = tk.Button(controls_frame, text="LANZAR RViz", 
                                 font=("Arial", 12, "bold"), bg="#2196F3", fg="white",
                                 command=self.launch_rviz, height=2)
        self.rviz_btn.pack(fill='x', pady=5)
        
        # Botón para detener RViz
        self.stop_rviz_btn = tk.Button(controls_frame, text="DETENER RViz", 
                                      font=("Arial", 10), bg="#f44336", fg="white",
                                      command=self.stop_rviz, state=tk.DISABLED)
        self.stop_rviz_btn.pack(fill='x', pady=5)
        
        # Estado de RViz
        self.rviz_status_label = tk.Label(controls_frame, text="RViz no iniciado", 
                                         font=("Arial", 10), fg="red")
        self.rviz_status_label.pack(pady=10)
        
        # El timer de actualización se inicia en __init__ después de crear
        # la barra global de pose TCP.

    def setup_tab4(self):
        """Configura la pestaña 4: Control por Pose Predefinida"""
        # Título
        title_label = tk.Label(
            self.tab4,
            text="Control por Pose Predefinida",
            font=("Arial", 14, "bold")
        )
        title_label.pack(pady=10)

        # Frame para todos los botones de pose
        buttons_frame = tk.Frame(self.tab4)
        buttons_frame.pack(fill='x', padx=40, pady=20)
        buttons_frame.columnconfigure(0, weight=1)
        buttons_frame.columnconfigure(1, weight=1)

        button_style = {
            "font": ("Arial", 11, "bold"),
            "bg": "#4CAF50",
            "fg": "white",
            "height": 2,
            "width": 28,
        }

        # Pose 1 personalizada
        self.pose1_btn = tk.Button(
            buttons_frame,
            text="Pose 1 personalizada: [15, 10, 90, 90, -50]",
            command=self.start_pose1_sequence,
            **button_style,
        )
        self.pose1_btn.grid(row=0, column=0, padx=5, pady=4, sticky='ew')

        # Pose 2 personalizada
        self.pose2_btn = tk.Button(
            buttons_frame,
            text="Pose 2 personalizada: [-90, -30, 120, 45, -60]",
            command=self.start_pose2_sequence,
            **button_style,
        )
        self.pose2_btn.grid(row=0, column=1, padx=5, pady=4, sticky='ew')

        # Poses de laboratorio
        self.pose_lab1_btn = tk.Button(
            buttons_frame,
            text="Pose Lab 1: [0, 0, 0, 0, 0]",
            command=self.start_lab_pose1_sequence,
            **button_style,
        )
        self.pose_lab1_btn.grid(row=1, column=0, padx=5, pady=4, sticky='ew')

        self.pose_lab2_btn = tk.Button(
            buttons_frame,
            text="Pose Lab 2: [25, 25, 20, -20, 0]",
            command=self.start_lab_pose2_sequence,
            **button_style,
        )
        self.pose_lab2_btn.grid(row=1, column=1, padx=5, pady=4, sticky='ew')

        self.pose_lab3_btn = tk.Button(
            buttons_frame,
            text="Pose Lab 3: [-35, 35, -30, 30, 0]",
            command=self.start_lab_pose3_sequence,
            **button_style,
        )
        self.pose_lab3_btn.grid(row=2, column=0, padx=5, pady=4, sticky='ew')

        self.pose_lab4_btn = tk.Button(
            buttons_frame,
            text="Pose Lab 4: [85, -20, 55, 25, 0]",
            command=self.start_lab_pose4_sequence,
            **button_style,
        )
        self.pose_lab4_btn.grid(row=2, column=1, padx=5, pady=4, sticky='ew')

        self.pose_lab5_btn = tk.Button(
            buttons_frame,
            text="Pose Lab 5: [80, -35, 55, -45, 0]",
            command=self.start_lab_pose5_sequence,
            **button_style,
        )
        self.pose_lab5_btn.grid(row=3, column=0, padx=5, pady=4, sticky='ew')

        # Etiqueta de estado general para las poses
        self.pose1_status_label = tk.Label(
            self.tab4,
            text="Listo",
            font=("Arial", 9),
            fg="green"
        )
        self.pose1_status_label.pack(pady=5)

    def setup_common_buttons(self):
        """Configura los botones comunes en la parte inferior"""
        # Frame para botones comunes
        common_buttons_frame = tk.Frame(self.window)
        common_buttons_frame.pack(fill='x', padx=20, pady=10)
        
        # Botón HOME
        home_btn = tk.Button(common_buttons_frame, text="HOME", 
                            font=("Arial", 10, "bold"), bg="#2196F3", fg="white",
                            command=self.home_all)
        home_btn.pack(side='left', padx=10)
        
        # Botón PARADA DE EMERGENCIA
        emergency_btn = tk.Button(common_buttons_frame, text="PARADA DE EMERGENCIA", 
                                 font=("Arial", 10, "bold"), bg="#f44336", fg="white",
                                 command=self.emergency_stop)
        emergency_btn.pack(side='right', padx=10)
        
        # Etiqueta de estado global
        self.status_label = tk.Label(common_buttons_frame, text="Sistema Listo", 
                                    font=("Arial", 9), fg="green")
        self.status_label.pack(side='bottom', pady=5)

    def setup_tcp_status_bar(self):
        """Barra inferior con XYZ y RPY del TCP (visible en casi todas las pestañas)."""
        self.tcp_status_frame = tk.Frame(self.window)
        self.tcp_status_frame.pack(fill='x', padx=20, pady=(0, 10))

        # Bloque de pose TCP
        tcp_title = tk.Label(
            self.tcp_status_frame,
            text="Pose TCP (X, Y, Z, R, P, Y):",
            font=("Arial", 9, "bold")
        )
        tcp_title.pack(side='left')

        tcp_values_frame = tk.Frame(self.tcp_status_frame)
        tcp_values_frame.pack(side='left', padx=8)

        self.tcp_x_label_global = tk.Label(tcp_values_frame, text="X: 0.000", font=("Arial", 9), width=10)
        self.tcp_x_label_global.pack(side='left')

        self.tcp_y_label_global = tk.Label(tcp_values_frame, text="Y: 0.000", font=("Arial", 9), width=10)
        self.tcp_y_label_global.pack(side='left')

        self.tcp_z_label_global = tk.Label(tcp_values_frame, text="Z: 0.000", font=("Arial", 9), width=10)
        self.tcp_z_label_global.pack(side='left')

        self.tcp_r_label_global = tk.Label(tcp_values_frame, text="R: 0.0°", font=("Arial", 9), width=10)
        self.tcp_r_label_global.pack(side='left')

        self.tcp_p_label_global = tk.Label(tcp_values_frame, text="P: 0.0°", font=("Arial", 9), width=10)
        self.tcp_p_label_global.pack(side='left')

        self.tcp_yaw_label_global = tk.Label(tcp_values_frame, text="Y: 0.0°", font=("Arial", 9), width=10)
        self.tcp_yaw_label_global.pack(side='left')

        # Separador vertical
        sep = tk.Frame(self.tcp_status_frame, width=2, bd=0, relief='sunken', bg="#cccccc")
        sep.pack(side='left', fill='y', padx=8)

        # Bloque de ángulos articulares (q1..q5)
        q_title = tk.Label(
            self.tcp_status_frame,
            text="Ángulos articulares (q1..q5, grados respecto a HOME):",
            font=("Arial", 9, "bold")
        )
        q_title.pack(side='left')

        q_values_frame = tk.Frame(self.tcp_status_frame)
        q_values_frame.pack(side='left', padx=8)

        self.q_labels_global = {}
        for i in range(1, 6):
            lbl = tk.Label(q_values_frame, text=f"q{i}: 0.0°", font=("Arial", 9), width=9)
            lbl.pack(side='left')
            self.q_labels_global[i] = lbl

    def setup_tab5(self):
        """Configura la pestaña 5: Información del grupo (Acerca de)"""
        info_frame = tk.Frame(self.tab5)
        info_frame.pack(fill='both', expand=True, padx=40, pady=40)

        title = tk.Label(
            info_frame,
            text="Proyecto: Lab 05 - Cinemática Directa - PhantomX Pincher X100",
            font=("Arial", 14, "bold")
        )
        title.pack(pady=(0, 20))

        group_label = tk.Label(
            info_frame,
            text=(
                "Integrantes del grupo:\n\n"
                "  • Sergio Andrés Bolaños Penagos\n"
                "  • Sergio Felipe Rodriguez Mayorga\n\n"
                "Universidad Nacional de Colombia\n"
                "Programa: Ingeniería Mecatrónica\n"
                "Asignatura: Robótica"
            ),
            justify=tk.LEFT,
            font=("Arial", 11),
        )
        group_label.pack(anchor='w')
    def setup_tab6(self):
        """Configura la pestaña 6: Control en espacio de la tarea (XYZ + RPY)."""
        frame = tk.Frame(self.tab6)
        frame.pack(fill='both', expand=True, padx=20, pady=20)

        title = tk.Label(
            frame,
            text="Control en Espacio de la Tarea (TCP - MoveIt)",
            font=("Arial", 14, "bold")
        )
        title.pack(pady=(0, 15))

        info = tk.Label(
            frame,
            text=(
                "Esta pestaña permite comandar el TCP en espacio cartesiano.\n"
                "Los ángulos Roll, Pitch y Yaw se ingresan en grados y se\n"
                "convierten internamente a radianes para MoveIt.\n"
                "El nodo 'commander' de MoveIt recibe el tópico 'pose_command'\n"
                "y planifica/ejecuta la trayectoria correspondiente."
            ),
            justify=tk.LEFT,
            font=("Arial", 9),
            bg="#f0f0f0",
            relief="solid",
            padx=10,
            pady=10,
        )
        info.pack(fill='x', pady=(0, 15))

        # Frame para campos numéricos
        fields = tk.Frame(frame)
        fields.pack(fill='x', pady=10)

        self.task_x_var = tk.DoubleVar(value=0.10)
        self.task_y_var = tk.DoubleVar(value=0.00)
        self.task_z_var = tk.DoubleVar(value=0.10)
        self.task_roll_deg_var = tk.DoubleVar(value=0.0)
        self.task_pitch_deg_var = tk.DoubleVar(value=0.0)
        self.task_yaw_deg_var = tk.DoubleVar(value=0.0)
        self.task_cartesian_var = tk.BooleanVar(value=False)

        def add_field(row, label_text, var, units):
            lbl = tk.Label(fields, text=label_text, font=("Arial", 10), width=10, anchor='e')
            lbl.grid(row=row, column=0, padx=5, pady=3)
            entry = tk.Entry(fields, textvariable=var, width=10, font=("Arial", 10))
            entry.grid(row=row, column=1, padx=5, pady=3)
            u_lbl = tk.Label(fields, text=units, font=("Arial", 9), anchor='w')
            u_lbl.grid(row=row, column=2, padx=5, pady=3, sticky='w')

        add_field(0, "X:", self.task_x_var, "m")
        add_field(1, "Y:", self.task_y_var, "m")
        add_field(2, "Z:", self.task_z_var, "m")
        add_field(3, "Roll:", self.task_roll_deg_var, "deg")
        add_field(4, "Pitch:", self.task_pitch_deg_var, "deg")
        add_field(5, "Yaw:", self.task_yaw_deg_var, "deg")

        # Checkbox de trayectoria cartesiana
        cartesian_chk = tk.Checkbutton(
            fields,
            text="Trayectoria cartesiana (cartesian_path=true)",
            variable=self.task_cartesian_var,
            font=("Arial", 9),
            anchor='w',
        )
        cartesian_chk.grid(row=6, column=0, columnspan=3, sticky='w', padx=5, pady=(10, 5))

        # Botón de envío
        send_btn = tk.Button(
            frame,
            text="ENVIAR POSE AL TCP (MoveIt)",
            font=("Arial", 11, "bold"),
            bg="#4CAF50",
            fg="white",
            command=self.send_task_space_pose,
            height=2,
            width=30,
        )
        send_btn.pack(pady=10)

        # Etiqueta de estado
        self.task_status_label = tk.Label(
            frame,
            text="Listo",
            font=("Arial", 9),
            fg="green",
        )
        self.task_status_label.pack(pady=5)

    def on_tab_changed(self, event):
        """Muestra u oculta la barra de pose TCP según la pestaña actual."""
        current_tab = self.notebook.select()
        tab_widget = self.notebook.nametowidget(current_tab)
        if tab_widget is self.tab5:
            # Ocultar barra TCP en pestaña "Acerca de"
            self.tcp_status_frame.pack_forget()
        else:
            # Volver a mostrar si no está ya empacada
            if not self.tcp_status_frame.winfo_ismapped():
                self.tcp_status_frame.pack(fill='x', padx=20, pady=(0, 10))

    def send_task_space_pose(self):
        """Publica un PoseCommand en el tópico pose_command para MoveIt."""
        # Verificar que el publisher exista (bindings de PoseCommand disponibles)
        if self.controller.pose_command_pub is None:
            self.task_status_label.config(
                text="pose_command no disponible (phantomx_pincher_interfaces sin soporte C)",
                fg="red",
            )
            return

        try:
            x = float(self.task_x_var.get())
            y = float(self.task_y_var.get())
            z = float(self.task_z_var.get())
            roll_deg = float(self.task_roll_deg_var.get())
            pitch_deg = float(self.task_pitch_deg_var.get())
            yaw_deg = float(self.task_yaw_deg_var.get())
        except ValueError:
            self.task_status_label.config(text="Error: valores numéricos inválidos", fg="red")
            return

        # Conversión a radianes para MoveIt
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)

        msg = PoseCommand()
        msg.x = x
        msg.y = y
        msg.z = z
        msg.roll = roll
        msg.pitch = pitch
        msg.yaw = yaw
        msg.cartesian_path = bool(self.task_cartesian_var.get())

        try:
            self.controller.pose_command_pub.publish(msg)
            self.task_status_label.config(
                text=f"Pose enviada: X={x:.3f}, Y={y:.3f}, Z={z:.3f}, "
                     f"R={roll_deg:.1f}°, P={pitch_deg:.1f}°, Y={yaw_deg:.1f}°",
                fg="blue",
            )
        except Exception as e:
            self.task_status_label.config(text=f"Error publicando pose: {e}", fg="red")

    def launch_rviz(self):
        """Lanza robot_state_publisher + RViz usando ros2 launch"""
        try:
            # Lanzar el launch file: pincher_description/display.launch.py
            cmd = ["ros2", "launch", "phantomx_pincher_description", "display.launch.py"]

            def run_rviz():
                # Este proceso levanta robot_state_publisher + rviz2
                self.rviz_process = subprocess.Popen(cmd)
                self.rviz_process.wait()
                # Cuando se cierra, actualizar la interfaz
                self.window.after(0, self.on_rviz_closed)

            thread = threading.Thread(target=run_rviz, daemon=True)
            thread.start()

            # Actualizar interfaz
            self.rviz_btn.config(state=tk.DISABLED, bg="#cccccc")
            self.stop_rviz_btn.config(state=tk.NORMAL, bg="#f44336")
            self.rviz_status_label.config(text="RViz + robot_state_publisher ejecutándose", fg="green")
            self.status_label.config(text="Lanzado view.launch.py (RViz + modelo)")
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo lanzar RViz: {str(e)}")
            self.rviz_status_label.config(text=f"Error: {str(e)}", fg="red")


    def stop_rviz(self):
        """Detiene el proceso de RViz"""
        if self.rviz_process:
            try:
                self.rviz_process.terminate()
                self.rviz_process = None
            except:
                pass
        
        self.on_rviz_closed()

    def on_rviz_closed(self):
        """Actualiza la interfaz cuando RViz se cierra"""
        self.rviz_btn.config(state=tk.NORMAL, bg="#2196F3")
        self.stop_rviz_btn.config(state=tk.DISABLED, bg="#cccccc")
        self.rviz_status_label.config(text="RViz no iniciado", fg="red")
        self.status_label.config(text="RViz cerrado")

    def get_rviz_config_path(self):
        """Obtiene la ruta al archivo de configuración de RViz"""
        # Intentar encontrar el paquete de descripción del robot
        try:
            # Si tienes un paquete de descripción del PhantomX Pincher instalado
            from ament_index_python.packages import get_package_share_directory
            pkg_path = get_package_share_directory('phantomx_pincher_description')
            return os.path.join(pkg_path, 'rviz', 'pincher.rviz')
        except:
            # Si no está instalado, usar configuración por defecto
            # En una implementación real, deberías tener este archivo
            return ""

    def update_joints_timer(self):
        """Actualiza periódicamente la pose TCP y los ángulos articulares en la interfaz."""
        joint_positions = getattr(self.controller, "current_joint_positions", [])

        # Actualizar pose TCP global (XYZ + RPY) y ángulos articulares (q1..q5) si están disponibles
        if hasattr(self.controller, "current_tcp_xyz") and hasattr(self.controller, "current_tcp_rpy"):
            x, y, z = self.controller.current_tcp_xyz
            roll, pitch, yaw = self.controller.current_tcp_rpy

            self.tcp_x_label_global.config(text=f"X: {x:.3f}")
            self.tcp_y_label_global.config(text=f"Y: {y:.3f}")
            self.tcp_z_label_global.config(text=f"Z: {z:.3f}")

            self.tcp_r_label_global.config(text=f"R: {math.degrees(roll):.1f}°")
            self.tcp_p_label_global.config(text=f"P: {math.degrees(pitch):.1f}°")
            self.tcp_yaw_label_global.config(text=f"Y: {math.degrees(yaw):.1f}°")

        # Actualizar q1..q5 en grados respecto a HOME usando current_joint_positions
        if joint_positions and hasattr(self, "q_labels_global"):
            for i in range(1, 6):
                if i - 1 < len(joint_positions):
                    angle_rad = joint_positions[i - 1]
                    angle_deg = math.degrees(angle_rad)
                    self.q_labels_global[i].config(text=f"q{i}: {angle_deg:.1f}°")

        # Programar siguiente actualización
        self.window.after(100, self.update_joints_timer)

    def start_pose_sequence(self, target_angles, sequence_name, button, status_label, reverse=False):
        """Inicia una secuencia genérica de pose.

        Por defecto va de base → extremo. Si reverse=True, va de extremo → base.
        """
        if self.pose_sequence_running:
            # Evitar lanzar múltiples secuencias simultáneas
            return

        # Comprobar parada de emergencia
        if self.controller.emergency_stop_activated:
            status_label.config(text="EMERGENCIA", fg="red")
            self.status_label.config(
                text="EMERGENCIA: No se puede mover motores",
                fg="red"
            )
            return

        # Usar la velocidad actual (slider de la pestaña 1)
        speed = self.speed_slider_tab1.get() if hasattr(self, "speed_slider_tab1") else 0
        if speed == 0:
            status_label.config(text="Velocidad 0", fg="orange")
            self.status_label.config(
                text="Velocidad 0: Los motores no se moverán",
                fg="orange"
            )
            return

        # Construir secuencia en el orden de dxl_ids (o inverso)
        motor_ids = list(self.controller.dxl_ids)
        if reverse:
            motor_ids = list(reversed(motor_ids))

        sequence = []
        for motor_id in motor_ids:
            if motor_id in target_angles:
                sequence.append((motor_id, target_angles[motor_id]))

        if not sequence:
            status_label.config(text="Sin motores", fg="red")
            return

        self.pose_sequence_running = True
        button.config(state=tk.DISABLED)
        status_label.config(text="Ejecutando...", fg="blue")
        self.status_label.config(text=f"Ejecutando {sequence_name}", fg="blue")

        # Iniciar ejecución escalonada usando after (no bloquea la GUI)
        self.run_pose_step(sequence, 0, sequence_name, button, status_label)

    def run_pose_step(self, sequence, index, sequence_name, button, status_label):
        """Ejecuta un paso de una secuencia de pose y programa el siguiente."""
        # Si se activó emergencia en medio de la secuencia, detenerla
        if self.controller.emergency_stop_activated:
            self.pose_sequence_running = False
            button.config(state=tk.NORMAL)
            status_label.config(text="EMERGENCIA", fg="red")
            self.status_label.config(
                text="EMERGENCIA: Secuencia detenida",
                fg="red"
            )
            return

        if index >= len(sequence):
            # Secuencia completada
            self.pose_sequence_running = False
            button.config(state=tk.NORMAL)
            status_label.config(text="Completado", fg="green")
            self.status_label.config(text=f"{sequence_name} completada", fg="green")
            self.window.after(
                3000,
                lambda: status_label.config(text="Listo", fg="green")
            )
            return

        motor_id, angle_deg = sequence[index]

        # Asegurarse de que el ángulo está dentro de los límites (NO se modifican los máximos)
        if angle_deg < MIN_ANGLE_DEG:
            angle_deg = MIN_ANGLE_DEG
        elif angle_deg > MAX_ANGLE_DEG:
            angle_deg = MAX_ANGLE_DEG

        dxl_position = self.controller.degrees_to_dxl(angle_deg, motor_id)
        self.controller.move_motor(motor_id, dxl_position)

        # Actualizar interfaz (sliders y entries si existen)
        if motor_id in self.sliders:
            self.sliders[motor_id].set(angle_deg)
            self.labels[motor_id].config(text=f'Pos: {angle_deg:.1f}°')

        if motor_id in self.entries:
            self.entries[motor_id].delete(0, tk.END)
            self.entries[motor_id].insert(0, f"{angle_deg:.1f}")
            self.entry_labels[motor_id].config(text="Enviado", fg="blue")

        self.status_label.config(
            text=f"Motor {motor_id} a {angle_deg:.1f}° ({sequence_name})",
            fg="blue"
        )

        # Programar el siguiente motor después de un pequeño retardo (ms)
        self.window.after(
            1000,
            lambda: self.run_pose_step(
                sequence,
                index + 1,
                sequence_name,
                button,
                status_label
            )
        )

    def start_pose1_sequence(self):
        """Inicia la secuencia para mover el robot a la Pose 1 en orden base → extremo."""
        target_angles = {
            1: 15.0,
            2: 10.0,
            3: 90.0,
            4: 90.0,
            5: -50.0,
        }
        self.start_pose_sequence(
            target_angles,
            "Pose 1",
            self.pose1_btn,
            self.pose1_status_label
        )

    def start_pose2_sequence(self):
        """Inicia la secuencia para mover el robot a la Pose 2 en orden base → extremo."""
        target_angles = {
            1: -90.0,
            2: -30.0,
            3: 120.0,
            4: 45.0,
            5: -60.0,
        }
        self.start_pose_sequence(
            target_angles,
            "Pose 2",
            self.pose2_btn,
            self.pose1_status_label
        )

    def start_home_pose_sequence(self):
        """Inicia la secuencia para mover el robot a la Pose HOME (alias de Pose Lab 1)."""
        self.start_lab_pose1_sequence()

    # ---------------- Poses del laboratorio (cinemática directa) ----------------

    def start_lab_pose1_sequence(self):
        """Pose Lab 1: [0, 0, 0, 0, 0] (equivalente a HOME)."""
        target_angles = {
            1: 0.0,
            2: 0.0,
            3: 0.0,
            4: 0.0,
            5: 0.0,
        }
        self.start_pose_sequence(
            target_angles,
            "Pose Lab 1",
            self.pose_lab1_btn,
            self.pose1_status_label,  # reutilizamos etiqueta principal
            reverse=True  # HOME: de la última articulación hacia la base
        )

    def start_lab_pose2_sequence(self):
        """Pose Lab 2: [25, 25, 20, -20, 0]."""
        target_angles = {
            1: 25.0,
            2: 25.0,
            3: 20.0,
            4: -20.0,
            5: 0.0,
        }
        self.start_pose_sequence(
            target_angles,
            "Pose Lab 2",
            self.pose_lab2_btn,
            self.pose1_status_label,
        )

    def start_lab_pose3_sequence(self):
        """Pose Lab 3: [-35, 35, -30, 30, 0]."""
        target_angles = {
            1: -35.0,
            2: 35.0,
            3: -30.0,
            4: 30.0,
            5: 0.0,
        }
        self.start_pose_sequence(
            target_angles,
            "Pose Lab 3",
            self.pose_lab3_btn,
            self.pose1_status_label,
        )

    def start_lab_pose4_sequence(self):
        """Pose Lab 4: [85, -20, 55, 25, 0]."""
        target_angles = {
            1: 85.0,
            2: -20.0,
            3: 55.0,
            4: 25.0,
            5: 0.0,
        }
        self.start_pose_sequence(
            target_angles,
            "Pose Lab 4",
            self.pose_lab4_btn,
            self.pose1_status_label,
        )

    def start_lab_pose5_sequence(self):
        """Pose Lab 5: [80, -35, 55, -45, 0]."""
        target_angles = {
            1: 80.0,
            2: -35.0,
            3: 55.0,
            4: -45.0,
            5: 0.0,
        }
        self.start_pose_sequence(
            target_angles,
            "Pose Lab 5",
            self.pose_lab5_btn,
            self.pose1_status_label,
        )

    def on_motor_slider_change(self, motor_id):
        """Se ejecuta CADA VEZ que se mueve el slider del motor (Pestaña 1).

        El slider está en grados, pero el motor sigue recibiendo ticks.
        """
        current_time = time.time()
        
        # Control de frecuencia de actualización para no saturar
        if current_time - self.last_motor_update[motor_id] >= self.update_interval:
            angle_deg = float(self.sliders[motor_id].get())
            
            # Solo mover si la velocidad no es 0 y no hay emergencia
            speed = self.speed_slider_tab1.get()
            if speed > 0 and not self.controller.emergency_stop_activated:
                dxl_position = self.controller.degrees_to_dxl(angle_deg, motor_id)
                self.controller.move_motor(motor_id, dxl_position)
                self.labels[motor_id].config(text=f'Pos: {angle_deg:.1f}°')
                self.last_motor_update[motor_id] = current_time
                
                # Actualizar estado brevemente
                self.status_label.config(text=f"Motor {motor_id} moviéndose a {angle_deg:.1f}°")
                self.window.after(2000, lambda: self.status_label.config(text="Sistema Listo", fg="green"))
            elif self.controller.emergency_stop_activated:
                self.status_label.config(text=f"EMERGENCIA: No se puede mover motor {motor_id}", fg="red")
            else:
                self.status_label.config(text=f"Velocidad 0: Motor {motor_id} no se moverá", fg="orange")

    def on_speed_slider_change(self, value):
        """Se ejecuta CADA VEZ que se mueve el slider de velocidad"""
        current_time = time.time()
        
        # Control de frecuencia de actualización
        if current_time - self.last_speed_update >= self.update_interval:
            speed = int(value)
            
            # Solo actualizar velocidad si no hay emergencia
            if not self.controller.emergency_stop_activated:
                self.controller.update_speed(speed)
                
                # Actualizar ambas etiquetas de velocidad
                self.speed_value_label_tab1.config(text=str(speed))
                self.speed_value_label_tab2.config(text=str(speed))
                
                # Sincronizar sliders de velocidad entre pestañas
                self.speed_slider_tab1.set(speed)
                self.speed_slider_tab2.set(speed)
                
                self.last_speed_update = current_time
                
                # Actualizar estado brevemente
                if speed == 0:
                    self.status_label.config(text=f"Velocidad 0: Los motores no se moverán", fg="orange")
                else:
                    self.status_label.config(text=f"Velocidad actualizada: {speed}")
                
                self.window.after(2000, lambda: self.status_label.config(text="Sistema Listo", fg="green"))
            else:
                self.status_label.config(text="EMERGENCIA: No se puede cambiar velocidad", fg="red")

    def move_single_motor_from_entry(self, motor_id):
        """Mueve un motor individual basado en el valor del entry (Pestaña 2, en grados)."""
        try:
            value = self.entries[motor_id].get()
            angle_deg = float(value)
            
            if MIN_ANGLE_DEG <= angle_deg <= MAX_ANGLE_DEG:
                # Verificar velocidad y estado de emergencia
                speed = self.speed_slider_tab2.get()
                if speed == 0:
                    self.entry_labels[motor_id].config(text="Velocidad 0", fg="orange")
                    self.status_label.config(text=f"Velocidad 0: Motor {motor_id} no se moverá", fg="orange")
                elif self.controller.emergency_stop_activated:
                    self.entry_labels[motor_id].config(text="EMERGENCIA", fg="red")
                    self.status_label.config(text="EMERGENCIA: No se puede mover motores", fg="red")
                else:
                    dxl_position = self.controller.degrees_to_dxl(angle_deg, motor_id)
                    self.controller.move_motor(motor_id, dxl_position)
                    self.entry_labels[motor_id].config(text="Enviado", fg="blue")
                    self.status_label.config(text=f"Motor {motor_id} moviéndose a {angle_deg:.1f}°", fg="blue")
                    
                    # Actualizar slider en la pestaña 1 si existe
                    if motor_id in self.sliders:
                        self.sliders[motor_id].set(angle_deg)
                    
                    # Resetear etiqueta después de 2 segundos
                    self.window.after(2000, lambda: self.entry_labels[motor_id].config(text="Listo", fg="green"))
                    self.window.after(2000, lambda: self.status_label.config(text="Sistema Listo", fg="green"))
            else:
                self.entry_labels[motor_id].config(
                    text=f"Error: {int(MIN_ANGLE_DEG)} a {int(MAX_ANGLE_DEG)}°",
                    fg="red"
                )
                
        except ValueError:
            self.entry_labels[motor_id].config(text="Error: Número", fg="red")

    def move_all_motors_from_entries(self):
        """Mueve todos los motores basado en los valores de los entries (Pestaña 2, en grados)."""
        # Verificar condiciones
        speed = self.speed_slider_tab2.get()
        if speed == 0:
            self.status_label.config(text="Velocidad 0: Los motores no se moverán", fg="orange")
            return
            
        if self.controller.emergency_stop_activated:
            self.status_label.config(text="EMERGENCIA: No se puede mover motores", fg="red")
            return
            
        success_count = 0
        for motor_id in self.controller.dxl_ids:
            try:
                value = self.entries[motor_id].get()
                angle_deg = float(value)
                
                if MIN_ANGLE_DEG <= angle_deg <= MAX_ANGLE_DEG:
                    dxl_position = self.controller.degrees_to_dxl(angle_deg, motor_id)
                    self.controller.move_motor(motor_id, dxl_position)
                    self.entry_labels[motor_id].config(text="Enviado", fg="blue")
                    success_count += 1
                    
                    # Actualizar slider en la pestaña 1 si existe
                    if motor_id in self.sliders:
                        self.sliders[motor_id].set(angle_deg)
                else:
                    self.entry_labels[motor_id].config(
                        text=f"Error: {int(MIN_ANGLE_DEG)} a {int(MAX_ANGLE_DEG)}°",
                        fg="red"
                    )
                    
            except ValueError:
                self.entry_labels[motor_id].config(text="Error: Número", fg="red")
        
        self.status_label.config(
            text=f"Comando enviado a {success_count}/{len(self.controller.dxl_ids)} motores",
            fg="blue"
        )
        
        # Resetear etiquetas después de 3 segundos
        self.window.after(
            3000,
            lambda: [label.config(text="Listo", fg="green") for label in self.entry_labels.values()]
        )
        self.window.after(3000, lambda: self.status_label.config(text="Sistema Listo", fg="green"))

    def home_all(self):
        """Mueve todos los motores a la posición HOME"""
        # Si hay parada de emergencia activada, mostrar confirmación para reactivar
        if self.controller.emergency_stop_activated:
            if messagebox.askokcancel("Reactivar Sistema", 
                                     "La parada de emergencia está activada.\n\n¿Desea reactivar el sistema y mover los motores a HOME?"):
                self.controller.home_all_motors()
                self.status_label.config(text="Sistema reactivado y motores movidos a HOME", fg="blue")
            else:
                return
        else:
            self.controller.home_all_motors()
            self.status_label.config(text="Todos los motores movidos a HOME", fg="blue")
        
        # Actualizar interfaz
        for motor_id in self.controller.dxl_ids:
            # Actualizar sliders en pestaña 1 (HOME = 0°)
            if motor_id in self.sliders:
                self.sliders[motor_id].set(0.0)
                self.labels[motor_id].config(text='Pos: 0.0°')
            
            # Actualizar entries en pestaña 2 (HOME = 0°)
            if motor_id in self.entries:
                self.entries[motor_id].delete(0, tk.END)
                self.entries[motor_id].insert(0, "0")
                self.entry_labels[motor_id].config(text="HOME", fg="green")
        
        self.window.after(3000, lambda: self.status_label.config(text="Sistema Listo", fg="green"))

    def emergency_stop(self):
        """Ejecuta parada de emergencia SIN confirmación"""
        self.controller.emergency_stop()
        self.status_label.config(text="PARADA DE EMERGENCIA ACTIVADA", fg="red")
        
        # Actualizar interfaz
        for label in self.entry_labels.values():
            label.config(text="EMERGENCIA", fg="red")

    def on_close(self):
        """Maneja el cierre de la ventana"""
        # Detener RViz si está ejecutándose
        if self.rviz_process:
            self.stop_rviz()
            
        if messagebox.askokcancel("Salir", "¿Estás seguro de que quieres salir?\nSe desactivará el torque de los motores."):
            self.controller.close()
            self.window.destroy()
            rclpy.shutdown()

    def run(self):
        """Ejecuta la interfaz"""
        try:
            self.window.mainloop()
        except KeyboardInterrupt:
            self.on_close()

def main(args=None):
    rclpy.init(args=args)

    # Crear el nodo controlador
    controller = PincherController()

    # Lanzar el spin de ROS2 en un hilo en segundo plano
    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(controller,),
        daemon=True
    )
    spin_thread.start()

    try:
        # Lanzar la GUI en el hilo principal
        gui = PincherGUI(controller)
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Cerrar todo ordenadamente
        controller.close()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()