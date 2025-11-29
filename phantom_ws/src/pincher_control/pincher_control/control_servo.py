import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
import time
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import subprocess
import os

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
        
        # Estado de emergencia
        self.emergency_stop_activated = False
        
        # Configuración inicial de los motores
        self.initialize_motors(goal_positions, moving_speed, torque_limit)
        
        # Publicador para Joint States (para RViz)
        from sensor_msgs.msg import JointState
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Timer para publicar joint states
        self.joint_state_timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz
        
        # Posiciones actuales de las articulaciones (en radianes / metros)
        # Mantén el tamaño sincronizado con el número de motores (dxl_ids)
        self.current_joint_positions = [0.0] * 5  # Para 5 articulaciones / ejes
        
        # Mapeo de IDs de motor a nombres de articulaciones del URDF de
        # `phantomx_pincher_description/urdf/phantomx_pincher.urdf`
        # Estos nombres DEBEN coincidir con los joints que publica
        # robot_state_publisher para que RViz pueda animar el modelo.
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

    def publish_joint_states(self):
        """Publica el estado de las articulaciones para RViz"""
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Header
        
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = "base_link"
        
        joint_state.name = self.joint_names
        joint_state.position = self.current_joint_positions
        
        self.joint_state_pub.publish(joint_state)

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
        
        # Crear notebook (pestañas)
        self.notebook = ttk.Notebook(self.window)
        self.notebook.pack(fill='both', expand=True, padx=10, pady=10)
        
        # Crear frames para cada pestaña
        self.tab1 = ttk.Frame(self.notebook)
        self.tab2 = ttk.Frame(self.notebook)
        self.tab3 = ttk.Frame(self.notebook)  # Nueva pestaña para RViz
        
        self.notebook.add(self.tab1, text='Control por Sliders')
        self.notebook.add(self.tab2, text='Control por Valores')
        self.notebook.add(self.tab3, text='Visualización RViz')
        
        # Configurar las pestañas
        self.setup_tab1()
        self.setup_tab2()
        self.setup_tab3()
        
        # Barra de botones comunes en la parte inferior
        self.setup_common_buttons()

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
        
        # Sliders para cada motor
        for i, motor_id in enumerate(self.controller.dxl_ids):
            motor_frame = tk.Frame(motors_frame)
            motor_frame.pack(fill='x', pady=5)
            
            # Label del motor
            motor_label = tk.Label(motor_frame, text=f'Motor {motor_id}', 
                                  font=("Arial", 10, "bold"), width=8)
            motor_label.pack(side='left', padx=5)
            
            # Slider
            slider = tk.Scale(motor_frame, from_=0, to=4095, orient=tk.HORIZONTAL, 
                             length=400, showvalue=True, resolution=1,
                             command=lambda value, mid=motor_id: self.on_motor_slider_change(mid))
            slider.set(DEFAULT_GOAL)
            slider.pack(side='left', fill='x', expand=True, padx=5)
            
            # Etiqueta de posición
            label = tk.Label(motor_frame, text=f'Pos: {DEFAULT_GOAL}', 
                            font=("Arial", 9), width=10)
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
        
        # Crear filas para cada motor
        for i, motor_id in enumerate(self.controller.dxl_ids):
            motor_frame = tk.Frame(motors_frame)
            motor_frame.pack(fill='x', pady=8)
            
            # Label del motor
            motor_label = tk.Label(motor_frame, text=f'Motor {motor_id}', 
                                  font=("Arial", 10, "bold"), width=8)
            motor_label.pack(side='left', padx=5)
            
            # Entry para valor en bits
            entry_label = tk.Label(motor_frame, text="Valor (0-4095):", 
                                  font=("Arial", 9))
            entry_label.pack(side='left', padx=5)
            
            entry = tk.Entry(motor_frame, width=8, font=("Arial", 10))
            entry.insert(0, str(DEFAULT_GOAL))
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
        
        # Frame de información de articulaciones
        joints_frame = tk.Frame(self.tab3)
        joints_frame.pack(fill='x', padx=20, pady=10)
        
        joints_label = tk.Label(joints_frame, text="Posiciones de Articulaciones (radianes):", 
                               font=("Arial", 10, "bold"))
        joints_label.pack(anchor='w')
        
        # Etiquetas para mostrar posiciones actuales
        self.joint_labels = {}
        for i, joint_name in enumerate(self.controller.joint_names):
            joint_frame = tk.Frame(joints_frame)
            joint_frame.pack(fill='x', pady=2)
            
            label = tk.Label(joint_frame, text=f"{joint_name}:", 
                            font=("Arial", 9), width=10, anchor='w')
            label.pack(side='left')
            
            value_label = tk.Label(joint_frame, text="0.000", 
                                  font=("Arial", 9), width=10)
            value_label.pack(side='left')
            
            self.joint_labels[joint_name] = value_label
        
        # Timer para actualizar las posiciones de las articulaciones
        self.update_joints_timer()

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

    def launch_rviz(self):
        """Lanza robot_state_publisher + RViz usando ros2 launch"""
        try:
            # Lanzar el launch file: pincher_description/view.launch.py
            cmd = ["ros2", "launch", "phantomx_pincher_description", "view.launch.py"]

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
        """Actualiza periódicamente las posiciones de las articulaciones en la interfaz"""
        for i, joint_name in enumerate(self.controller.joint_names):
            if i < len(self.controller.current_joint_positions):
                position = self.controller.current_joint_positions[i]
                self.joint_labels[joint_name].config(text=f"{position:.3f}")
        
        # Programar siguiente actualización
        self.window.after(100, self.update_joints_timer)

    def on_motor_slider_change(self, motor_id):
        """Se ejecuta CADA VEZ que se mueve el slider del motor (Pestaña 1)"""
        current_time = time.time()
        
        # Control de frecuencia de actualización para no saturar
        if current_time - self.last_motor_update[motor_id] >= self.update_interval:
            position = self.sliders[motor_id].get()
            
            # Solo mover si la velocidad no es 0 y no hay emergencia
            speed = self.speed_slider_tab1.get()
            if speed > 0 and not self.controller.emergency_stop_activated:
                self.controller.move_motor(motor_id, position)
                self.labels[motor_id].config(text=f'Pos: {position}')
                self.last_motor_update[motor_id] = current_time
                
                # Actualizar estado brevemente
                self.status_label.config(text=f"Motor {motor_id} moviéndose a {position}")
                self.window.after(2000, lambda: self.status_label.config(text="Sistema Listo"))
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
        """Mueve un motor individual basado en el valor del entry (Pestaña 2)"""
        try:
            value = self.entries[motor_id].get()
            position = int(value)
            
            if 0 <= position <= 4095:
                # Verificar velocidad y estado de emergencia
                speed = self.speed_slider_tab2.get()
                if speed == 0:
                    self.entry_labels[motor_id].config(text="Velocidad 0", fg="orange")
                    self.status_label.config(text=f"Velocidad 0: Motor {motor_id} no se moverá", fg="orange")
                elif self.controller.emergency_stop_activated:
                    self.entry_labels[motor_id].config(text="EMERGENCIA", fg="red")
                    self.status_label.config(text="EMERGENCIA: No se puede mover motores", fg="red")
                else:
                    self.controller.move_motor(motor_id, position)
                    self.entry_labels[motor_id].config(text="Enviado", fg="blue")
                    self.status_label.config(text=f"Motor {motor_id} moviéndose a {position}")
                    
                    # Actualizar slider en la pestaña 1 si existe
                    if motor_id in self.sliders:
                        self.sliders[motor_id].set(position)
                    
                    # Resetear etiqueta después de 2 segundos
                    self.window.after(2000, lambda: self.entry_labels[motor_id].config(text="Listo", fg="green"))
                    self.window.after(2000, lambda: self.status_label.config(text="Sistema Listo"))
            else:
                self.entry_labels[motor_id].config(text="Error: 0-4095", fg="red")
                
        except ValueError:
            self.entry_labels[motor_id].config(text="Error: Número", fg="red")

    def move_all_motors_from_entries(self):
        """Mueve todos los motores basado en los valores de los entries (Pestaña 2)"""
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
                position = int(value)
                
                if 0 <= position <= 4095:
                    self.controller.move_motor(motor_id, position)
                    self.entry_labels[motor_id].config(text="Enviado", fg="blue")
                    success_count += 1
                    
                    # Actualizar slider en la pestaña 1 si existe
                    if motor_id in self.sliders:
                        self.sliders[motor_id].set(position)
                else:
                    self.entry_labels[motor_id].config(text="Error: 0-4095", fg="red")
                    
            except ValueError:
                self.entry_labels[motor_id].config(text="Error: Número", fg="red")
        
        self.status_label.config(text=f"Comando enviado a {success_count}/{len(self.controller.dxl_ids)} motores")
        
        # Resetear etiquetas después de 3 segundos
        self.window.after(3000, lambda: [label.config(text="Listo", fg="green") 
                                        for label in self.entry_labels.values()])
        self.window.after(3000, lambda: self.status_label.config(text="Sistema Listo"))

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
            # Actualizar sliders en pestaña 1
            if motor_id in self.sliders:
                self.sliders[motor_id].set(DEFAULT_GOAL)
                self.labels[motor_id].config(text=f'Pos: {DEFAULT_GOAL}')
            
            # Actualizar entries en pestaña 2
            if motor_id in self.entries:
                self.entries[motor_id].delete(0, tk.END)
                self.entries[motor_id].insert(0, str(DEFAULT_GOAL))
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