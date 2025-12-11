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

    Al recibir "rectangle" publicará en `/pose_command` la pose:
        x=0.098, y=-0.001, z=0.049,
        roll=3.142, pitch=0.007, yaw=-0.007
    """

    def __init__(self) -> None:
        super().__init__('shape_to_pose_node')

        # Publisher hacia MoveIt / commander
        self.pose_pub = self.create_publisher(PoseCommand, 'pose_command', 10)

        # Suscriptor del tipo de figura
        self.create_subscription(String, 'figure_type', self.figure_callback, 10)

        # Diccionario: nombre de figura -> (x, y, z, roll, pitch, yaw)
        # Puedes añadir aquí más figuras/poses según las vayas midiendo.
        self.shape_to_pose = {
            # Rectángulo: valores proporcionados por ti
            'rectangle': (0.098, -0.001, 0.049, 3.142, 0.007, -0.007),
            'rectangulo': (0.098, -0.001, 0.049, 3.142, 0.007, -0.007),
            'rectángulo': (0.098, -0.001, 0.049, 3.142, 0.007, -0.007),
        }

        self.get_logger().info(
            'ShapeToPoseNode inicializado. Publique en /figure_type (String) valores '
            "como 'rectangle', y se enviará la PoseCommand correspondiente a /pose_command."
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

        cmd = PoseCommand()
        cmd.x = float(x)
        cmd.y = float(y)
        cmd.z = float(z)
        cmd.roll = float(roll)
        cmd.pitch = float(pitch)
        cmd.yaw = float(yaw)
        cmd.cartesian_path = False

        self.pose_pub.publish(cmd)
        self.get_logger().info(
            f"Enviando PoseCommand para figura '{name}': "
            f"x={x:.3f}, y={y:.3f}, z={z:.3f}, "
            f"roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}"
        )


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
