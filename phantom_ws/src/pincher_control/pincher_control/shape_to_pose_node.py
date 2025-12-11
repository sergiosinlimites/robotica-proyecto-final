import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from phantomx_pincher_interfaces.msg import PoseCommand


class ShapeToPoseNode(Node):
    """Nodo sencillo que mapea el tipo de figura a una PoseCommand fija.

    Uso esperado:
      - Suscribe:  `/figure_type` (std_msgs/String)
      - Publica:   `/pose_command` (PoseCommand)

    Ejemplo de uso desde consola:

        ros2 run pincher_control shape_to_pose

        # En otra terminal
        ros2 topic pub -1 /figure_type std_msgs/msg/String "data: 'rectangle'"

    """

    def __init__(self) -> None:
        super().__init__('shape_to_pose_node')

        # Publisher hacia MoveIt / commander
        self.pose_pub = self.create_publisher(PoseCommand, 'pose_command', 10)

        # Suscriptor del tipo de figura
        self.create_subscription(String, 'figure_type', self.figure_callback, 10)

        # Diccionario: nombre de figura -> (x, y, z, roll, pitch, yaw)
        #
        # - 'recolectar' es la pose de recolección que ya probaste:
        #     x=0.128, y=0.0, z=0.100, roll=3.142, pitch=0.0, yaw=0.0
        # - 'pose2' es la segunda pose que definiste sobre la zona:
        #     x=0.185, y=-0.018, z=0.243, roll=0.0, pitch=-1.461, yaw=3.046
        # - 'cubo', 'cilindro', 'rectangulo', 'pentagono' son las poses de
        #   cada caneca que mediste con tf2_echo.
        self.shape_to_pose = {
            # Pose de recolección en la zona (antes llamada 'rectangle')
            'recolectar': (0.128, 0.0, 0.100, 3.142, 0.0, 0.0),

            # Pose 2 (sobre la zona de recolección)
            'pose2': (0.185, -0.018, 0.243, 0.0, -1.461, 3.046),

            # Caneca del cubo (roja)
            'cubo': (-0.009, 0.117, 0.104, 3.142, -0.074, 1.650),
            'cube': (-0.009, 0.117, 0.104, 3.142, -0.074, 1.650),

            # Caneca del cilindro (verde)
            'cilindro': (0.196, 0.091, 0.179, 3.142, -0.957, 0.433),
            'cylinder': (0.196, 0.091, 0.179, 3.142, -0.957, 0.433),

            # Caneca del rectángulo (amarilla)
            'rectangulo': (-0.010, -0.122, 0.109, 3.142, 0.087, -1.650),
            'rectángulo': (-0.010, -0.122, 0.109, 3.142, 0.087, -1.650),
            'rectangle': (-0.010, -0.122, 0.109, 3.142, 0.087, -1.650),

            # Caneca del pentágono (azul)
            'pentagono': (0.192, -0.088, 0.184, 3.142, -1.105, -0.433),
            'pentágono': (0.192, -0.088, 0.184, 3.142, -1.105, -0.433),
            'pentagon': (0.192, -0.088, 0.184, 3.142, -1.105, -0.433),
        }

        self.get_logger().info(
            'ShapeToPoseNode inicializado. Publique en /figure_type (String) valores '
            "como 'recolectar', 'cubo', 'cilindro', 'rectangulo', 'pentagono', etc."
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def figure_callback(self, msg: String) -> None:
        name = msg.data.strip().lower()
        if name not in self.shape_to_pose:
            self.get_logger().warn(
                f"Figura '{msg.data}' desconocida. Figuras válidas: {list(self.shape_to_pose.keys())}"
            )
            return

        x, y, z, roll, pitch, yaw = self.shape_to_pose[name]

        # Ir directamente a la pose objetivo de la figura (sin pasar por HOME)
        cmd = self._build_pose_command(x, y, z, roll, pitch, yaw)
        self.pose_pub.publish(cmd)
        self.get_logger().info(
            f"Enviando PoseCommand para figura '{name}': "
            f"x={x:.3f}, y={y:.3f}, z={z:.3f}, "
            f"roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}"
        )

    def _build_pose_command(
        self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float
    ) -> PoseCommand:
        """Construye un PoseCommand a partir de valores escalares."""
        cmd = PoseCommand()
        cmd.x = float(x)
        cmd.y = float(y)
        cmd.z = float(z)
        cmd.roll = float(roll)
        cmd.pitch = float(pitch)
        cmd.yaw = float(yaw)
        cmd.cartesian_path = False
        return cmd


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ShapeToPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
