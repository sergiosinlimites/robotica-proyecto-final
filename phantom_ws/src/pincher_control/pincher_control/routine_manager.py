#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped
from phantomx_pincher_interfaces.msg import PoseCommand
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
import yaml
import os
import math
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Bool
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from example_interfaces.msg import Float64MultiArray

class RoutineManager(Node):
    def __init__(self):
        super().__init__('routine_manager')
        
        self.declare_parameter('routines_file', 'routines.yaml')
        self.routines_file = self.get_parameter('routines_file').get_parameter_value().string_value
        
        self.declare_parameter('target_routine', 'recoleccion')
        self.declare_parameter('step_timeout_sec', 20.0)
        self.declare_parameter('republish_interval_sec', 0.0)  # 0 = no republish (recomendado)
        self.declare_parameter('reach_tolerance_m', 0.012)
        self.declare_parameter('require_vacio_between_same_routine', True)
        self.declare_parameter('vacio_hold_sec', 0.8)
        self.declare_parameter('home_joint_tolerance_rad', 0.12)

        # Load routines
        self.routines = self.load_routines()
        
        # Publisher for pose commands
        self.pose_pub = self.create_publisher(PoseCommand, '/pose_command', 10)
        
        # Publisher for gripper commands
        self.gripper_pub = self.create_publisher(Bool, '/set_gripper', 10)
        
        # TF Buffer for checking current pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Service to start a routine
        self.srv = self.create_service(Trigger, 'start_routine', self.start_routine_callback)

        # Vision trigger: /figure_type -> ejecuta rutina del mismo nombre (cubo/cilindro/pentagono/rectangulo)
        # Esto permite que el nodo YOLO solo publique la clase y aquí se ejecute el YAML completo.
        self.figure_sub = self.create_subscription(String, "/figure_type", self.figure_callback, 10)
        # Estado continuo: vacio/unknown/<clase>
        self.figure_state_sub = self.create_subscription(String, "/figure_state", self.figure_state_callback, 10)

        # Joint states (para confirmar HOME = joints ~ 0)
        self.joint_state_sub = self.create_subscription(JointState, "joint_states", self.joint_state_callback, 10)
        self.latest_joint_positions = {}
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, "joint_command", 10)
        
        self.current_routine = []
        self.current_step_index = 0
        self.is_executing = False
        self.target_pose = None
        self.step_started_time = 0.0
        self.last_pub_time = 0.0
        self.step_published = False
        self.pause_until = 0.0
        self.recovery_active = False
        self.last_completed_figure = ""
        self.last_started_figure = ""
        self.awaiting_vacio = False
        self.last_vacio_time = 0.0
        self.vacio_is_stable = False
        self.ensure_home_active = False
        self.ensure_home_sent = False
        
        # Timer for execution loop
        self.timer = self.create_timer(0.1, self.execution_loop)
        
        self.get_logger().info('Routine Manager initialized')

    def figure_callback(self, msg: String):
        figure = msg.data.lower().strip()
        if figure in ("", "unknown", "vacio"):
            return

        if self.is_executing:
            # Mientras ejecuta una rutina, ignorar nuevas detecciones
            return

        # No repetir la misma rutina hasta que se haya visto "vacio" (objeto retirado)
        require_vacio = bool(self.get_parameter('require_vacio_between_same_routine').value)
        if require_vacio and self.awaiting_vacio and figure == self.last_completed_figure:
            return

        # Además, exigir que "vacio" se haya observado recientemente antes de disparar la primera vez
        if require_vacio and self.last_completed_figure and (figure == self.last_completed_figure) and (not self.vacio_is_stable):
            return

        if figure not in self.routines:
            self.get_logger().warn(f"Rutina '{figure}' no existe en YAML. Disponibles: {list(self.routines.keys())}")
            return

        self.current_routine = self.routines[figure]
        self.current_step_index = 0
        self.is_executing = True
        self.last_started_figure = figure
        now = self.get_clock().now().nanoseconds / 1e9
        self.step_started_time = now
        self.last_pub_time = 0.0
        self.step_published = False
        self.pause_until = 0.0
        self.recovery_active = False
        self.ensure_home_active = False
        self.ensure_home_sent = False
        self.get_logger().info(f"[VISION] Starting routine: {figure}")

    def figure_state_callback(self, msg: String):
        state = msg.data.lower().strip()
        now = self.get_clock().now().nanoseconds / 1e9
        if state == "vacio":
            self.last_vacio_time = now
            # Considerar vacio estable tras un pequeño hold
            self.vacio_is_stable = True
            if self.awaiting_vacio:
                # ya se retiró el objeto, permitir próxima rutina (incluso misma clase)
                self.awaiting_vacio = False
        else:
            # cualquier cosa distinta a vacio invalida el "vacio estable" pasado un tiempo
            hold = float(self.get_parameter('vacio_hold_sec').value)
            if hold <= 0.0:
                self.vacio_is_stable = False
            else:
                if (now - self.last_vacio_time) > hold:
                    self.vacio_is_stable = False

    def joint_state_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.latest_joint_positions[name] = pos

    def load_routines(self):
        # Try to find the file in the package share directory first
        try:
            pkg_share = get_package_share_directory('pincher_control')
            config_path = os.path.join(pkg_share, 'config', self.routines_file)
            if not os.path.exists(config_path):
                # Fallback to local path if running from source (for testing)
                config_path = self.routines_file
                
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)['routines']
        except Exception as e:
            self.get_logger().error(f'Failed to load routines: {e}')
            return {}

    def start_routine_callback(self, request, response):
        routine_name = self.get_parameter('target_routine').get_parameter_value().string_value
        
        if routine_name not in self.routines:
            response.success = False
            response.message = f"Routine '{routine_name}' not found. Available: {list(self.routines.keys())}"
            return response
            
        self.current_routine = self.routines[routine_name]
        self.current_step_index = 0
        self.is_executing = True
        self.get_logger().info(f"Starting routine: {routine_name}")
        self.last_started_figure = routine_name
        now = self.get_clock().now().nanoseconds / 1e9
        self.step_started_time = now
        self.last_pub_time = 0.0
        self.step_published = False
        self.pause_until = 0.0
        self.recovery_active = False
        
        response.success = True
        response.message = f"Started routine {routine_name}"
        return response

    def execution_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9

        if not self.is_executing or self.current_step_index >= len(self.current_routine):
            if self.is_executing:
                self.get_logger().info("Routine completed")
            self._reset_state()
            return

        # pause window (for gripper settling or sequencing) - NEVER block with sleep()
        if now < self.pause_until:
            return

        step = self.current_routine[self.current_step_index]
        target = step['pose']
        
        # Check if we reached the target
        if self.check_reached(target, tolerance=float(self.get_parameter('reach_tolerance_m').value)):
            self.get_logger().info(f"Step {step['name']} reached")
            
            # Handle Gripper Action (Post-move)
            if 'gripper' in step:
                action = step['gripper']
                msg = Bool()
                if action == 'open':
                    msg.data = True
                    self.get_logger().info("Opening gripper...")
                elif action == 'close' or action == 'closed':
                    msg.data = False
                    self.get_logger().info("Closing gripper...")
                
                self.gripper_pub.publish(msg)
                # Dejar tiempo para que el gripper termine sin bloquear el executor
                self.pause_until = now + 0.6
            
            self.current_step_index += 1
            # Reiniciar estado de publicación por paso
            self.step_started_time = now
            self.last_pub_time = 0.0
            self.step_published = False

            if self.current_step_index >= len(self.current_routine):
                self.get_logger().info("Routine completed")
                # Guardar última figura ejecutada y exigir vacio antes de repetir
                if self.last_started_figure:
                    self.last_completed_figure = self.last_started_figure
                    self.awaiting_vacio = bool(self.get_parameter('require_vacio_between_same_routine').value)
                self._reset_state()
                return

            # pequeño retardo entre pasos (sin sleep)
            self.pause_until = max(self.pause_until, now + 0.8)
            return

        # Si el paso actual es HOME, al menos una vez intentamos alinear joints a 0 si no están
        if self._is_home_pose(target) and not self.ensure_home_active:
            self.ensure_home_active = True
            self.ensure_home_sent = False

        # Timeout por paso: si no llega, forzar HOME (joints 0 via heurística del commander)
        step_timeout = float(self.get_parameter('step_timeout_sec').value)
        if step_timeout > 0 and (now - self.step_started_time) > step_timeout:
            self.get_logger().warn(
                f"Timeout en paso '{step.get('name','?')}'. Forzando HOME para recuperar."
            )
            self._recover_to_home(now)
            return

        # Publicar target: idealmente UNA vez por paso
        republish_interval = float(self.get_parameter('republish_interval_sec').value)
        should_publish = (not self.step_published) or (
            republish_interval > 0 and (now - self.last_pub_time) >= republish_interval
        )
        if should_publish:
            msg = PoseCommand()
            msg.x = float(target['x'])
            msg.y = float(target['y'])
            msg.z = float(target['z'])
            msg.roll = float(target['roll'])
            msg.pitch = float(target['pitch'])
            msg.yaw = float(target['yaw'])
            msg.cartesian_path = False
            self.pose_pub.publish(msg)
            self.last_pub_time = now
            self.step_published = True

        # Post-check HOME: si estamos en HOME (por TF) pero joints no están cerca de cero -> mandar joint_command [0,0,0,0]
        if self.ensure_home_active:
            if self._home_joints_are_zero():
                self.ensure_home_active = False
                self.ensure_home_sent = False
            else:
                if not self.ensure_home_sent:
                    self._send_home_joint_command()
                    self.ensure_home_sent = True


    def _recover_to_home(self, now: float):
        """Publica HOME (pose 'home' de routines.yaml) y aborta la rutina actual cuando llegue."""
        self.recovery_active = True
        self.current_step_index = 0
        self.current_routine = [
            {
                "name": "RecoverHome",
                "pose": {"x": 0.0, "y": 0.0, "z": 0.406, "roll": 0.0, "pitch": 0.0, "yaw": 3.142},
            }
        ]
        self.step_started_time = now
        self.last_pub_time = 0.0
        self.step_published = False
        self.pause_until = now + 0.2

    def _reset_state(self):
        self.current_routine = []
        self.current_step_index = 0
        self.is_executing = False
        self.target_pose = None
        self.step_started_time = 0.0
        self.last_pub_time = 0.0
        self.step_published = False
        self.pause_until = 0.0
        self.recovery_active = False
        self.ensure_home_active = False
        self.ensure_home_sent = False
        self.last_started_figure = ""

    def check_reached(self, target, tolerance=0.01):
        try:
            # Get current pose of end effector relative to base
            # Assuming 'phantomx_pincher_base_link' to 'phantomx_pincher_end_effector'
            # Adjust frame names as per your TF tree
            trans = self.tf_buffer.lookup_transform(
                'phantomx_pincher_base_link',
                'phantomx_pincher_end_effector',
                rclpy.time.Time())
            
            dx = trans.transform.translation.x - target['x']
            dy = trans.transform.translation.y - target['y']
            dz = trans.transform.translation.z - target['z']
            
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            # We can also check orientation if needed, but for 4-DOF maybe just position
            return dist < tolerance
            
        except Exception as e:
            # self.get_logger().warn(f"TF Error: {e}")
            return False

    def _is_home_pose(self, target) -> bool:
        return abs(float(target.get("x", 0.0))) < 0.05 and abs(float(target.get("y", 0.0))) < 0.05 and float(target.get("z", 0.0)) > 0.35

    def _home_joints_are_zero(self) -> bool:
        tol = float(self.get_parameter('home_joint_tolerance_rad').value)
        names = [
            "phantomx_pincher_arm_shoulder_pan_joint",
            "phantomx_pincher_arm_shoulder_lift_joint",
            "phantomx_pincher_arm_elbow_flex_joint",
            "phantomx_pincher_arm_wrist_flex_joint",
        ]
        vals = [self.latest_joint_positions.get(n, None) for n in names]
        if any(v is None for v in vals):
            return False
        return all(abs(v) <= tol for v in vals)

    def _send_home_joint_command(self):
        msg = Float64MultiArray()
        # commander acepta >=4; enviamos q1..q4
        msg.data = [0.0, 0.0, 0.0, 0.0]
        self.get_logger().info("HOME joints not zero -> enviando joint_command [0,0,0,0] para alinear.")
        self.joint_cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RoutineManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
