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
import time
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Bool

class RoutineManager(Node):
    def __init__(self):
        super().__init__('routine_manager')
        
        self.declare_parameter('routines_file', 'routines.yaml')
        self.routines_file = self.get_parameter('routines_file').get_parameter_value().string_value
        
        self.declare_parameter('target_routine', 'recoleccion')

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
        
        self.current_routine = []
        self.current_step_index = 0
        self.is_executing = False
        self.target_pose = None
        self.last_pub_time = 0.0
        
        # Timer for execution loop
        self.timer = self.create_timer(0.1, self.execution_loop)
        
        self.get_logger().info('Routine Manager initialized')

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
        
        response.success = True
        response.message = f"Started routine {routine_name}"
        return response

    def execution_loop(self):
        if not self.is_executing or self.current_step_index >= len(self.current_routine):
            if self.is_executing: # This means it was executing but now finished
                self.get_logger().info("Routine completed")
            self.is_executing = False
            return

        step = self.current_routine[self.current_step_index]
        target = step['pose']
        
        # Check if we reached the target
        if self.check_reached(target):
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
                time.sleep(0.5) # Wait for gripper to move (fast now)
            
            self.current_step_index += 1
            if self.current_step_index >= len(self.current_routine):
                self.get_logger().info("Routine completed")
                self.is_executing = False
            else:
                # Wait a bit before next move?
                time.sleep(1.0) 
            return # Return after processing a step to avoid publishing new pose immediately

        # Publish command continuously (every 2.0 seconds to allow planning time)
        # User said: "constantly publish that pose... and at some point it will succeed"
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_pub_time > 2.0:
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
            # self.get_logger().info(f"Publishing target: {step['name']}")

        # Check if reached
        if self.check_reached(target):
            self.get_logger().info(f"Reached target: {step['name']}")
            self.current_step_index += 1
            # Wait a bit before next move?
            time.sleep(1.0) 

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

def main(args=None):
    rclpy.init(args=args)
    node = RoutineManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
