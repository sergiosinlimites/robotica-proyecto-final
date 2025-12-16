import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


# Orden canónico de articulaciones según el URDF y el controlador
JOINT_ORDER = [
    'phantomx_pincher_arm_shoulder_pan_joint',   # 1 - base
    'phantomx_pincher_arm_shoulder_lift_joint',  # 2 - shoulder
    'phantomx_pincher_arm_elbow_flex_joint',     # 3 - elbow
    'phantomx_pincher_arm_wrist_flex_joint',     # 4 - wrist
    'phantomx_pincher_gripper_finger1_joint',    # 5 - gripper
]


class JointAnglesDegreesNode(Node):
    """Nodo que lee /joint_states y muestra los 5 ángulos en grados respecto a HOME."""

    def __init__(self):
        super().__init__('joint_angles_degrees_node')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
        )

        # Mensaje inicial aclarando el orden
        order_msg_lines = [
            "Leyendo /joint_states y mostrando ángulos en grados respecto a HOME.",
            "Orden de articulaciones (1 → 5):",
            f"  1: base     -> {JOINT_ORDER[0]}",
            f"  2: shoulder -> {JOINT_ORDER[1]}",
            f"  3: elbow    -> {JOINT_ORDER[2]}",
            f"  4: wrist    -> {JOINT_ORDER[3]}",
            f"  5: gripper  -> {JOINT_ORDER[4]}",
            "",
            "Formato de salida:",
            "  [1: base, 2: shoulder, 3: elbow, 4: wrist, 5: gripper] =",
            "  [a1_deg, a2_deg, a3_deg, a4_deg, a5_deg]",
        ]
        self.get_logger().info("\n".join(order_msg_lines))

    def joint_state_callback(self, msg: JointState) -> None:
        """Convierte las posiciones a grados y las imprime en el orden definido."""
        if not msg.name or not msg.position:
            return

        # Construir un mapa nombre -> posición (rad)
        name_to_pos = {}
        for name, pos in zip(msg.name, msg.position):
            name_to_pos[name] = pos

        angles_deg = []
        for joint_name in JOINT_ORDER:
            if joint_name not in name_to_pos:
                # Si aún no tenemos todas las articulaciones, esperamos a un mensaje más completo
                self.get_logger().throttle(
                    self.get_clock(),
                    5000.0,
                    f"Esperando todas las articulaciones en /joint_states. Falta: {joint_name}",
                )
                return
            rad = name_to_pos[joint_name]
            angles_deg.append(rad * 180.0 / math.pi)

        a1, a2, a3, a4, a5 = angles_deg

        # Imprimir en una sola línea clara
        self.get_logger().info(
            "[1: base, 2: shoulder, 3: elbow, 4: wrist, 5: gripper] = "
            f"[{a1:7.2f}°, {a2:7.2f}°, {a3:7.2f}°, {a4:7.2f}°, {a5:7.2f}°]"
        )


def main(args=None):
    rclpy.init(args=args)
    node = JointAnglesDegreesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


