import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from phantomx_pincher_interfaces.msg import PoseCommand


class ObjectSortingNode(Node):
    """
    Nodo que implementa la rutina de clasificación de objetos usando MoveIt.

    - Suscribe:
        * 'figure_type' (std_msgs/String): tipo de figura en la zona de recolección
          valores esperados: 'cubo', 'cilindro', 'pentagono', 'rectangulo'
        * 'pickup_pose' (PoseCommand): pose en la zona de recolección
        * 'bin_red_pose', 'bin_green_pose', 'bin_blue_pose', 'bin_yellow_pose' (PoseCommand):
          poses de las canecas roja, verde, azul y amarilla
    - Publica:
        * 'pose_command' (PoseCommand): comandos de posición para el brazo (MoveIt)
        * 'open_gripper' (std_msgs/Bool): abre/cierra el gripper (true = abrir, false = cerrar)

    Flujo (simplificado):
        1. Recibe/almacena todas las poses necesarias.
        2. Al recibir 'figure_type', selecciona la caneca correcta.
        3. Ejecuta la secuencia:
            - ir sobre la zona de recolección
            - bajar a recoger
            - cerrar gripper
            - subir
            - ir sobre la caneca correspondiente
            - bajar a depositar
            - abrir gripper
            - subir (posición segura)

    NOTA IMPORTANTE:
        Este nodo asume que:
        - Ya está corriendo `phantomx_pincher_bringup` + `commander` (MoveIt).
        - Las poses publicadas en 'pickup_pose' y 'bin_*_pose' están en el mismo
          frame que usa MoveIt (por defecto, el planning frame del grupo "arm").
        - Los valores numéricos (x, y, z, roll, pitch, yaw) se obtuvieron con
          RViz + tf2_echo como se indica en la guía.
    """

    def __init__(self) -> None:
        super().__init__("object_sorting_node")

        # Offset vertical para "pose sobre el punto" (en metros)
        self.approach_offset_z = 0.05

        # Últimos valores recibidos
        self.figure_type: Optional[str] = None
        self.pickup_pose: Optional[PoseCommand] = None
        self.bin_poses: Dict[str, Optional[PoseCommand]] = {
            "red": None,
            "green": None,
            "blue": None,
            "yellow": None,
        }

        # Para evitar ejecutar la rutina infinitas veces
        self.run_once = False

        # Publishers
        self.pose_pub = self.create_publisher(PoseCommand, "pose_command", 10)
        self.gripper_pub = self.create_publisher(Bool, "open_gripper", 10)

        # Subscribers
        self.create_subscription(String, "figure_type", self.figure_type_callback, 10)
        self.create_subscription(PoseCommand, "pickup_pose", self.pickup_pose_callback, 10)
        self.create_subscription(PoseCommand, "bin_red_pose", self._make_bin_callback("red"), 10)
        self.create_subscription(PoseCommand, "bin_green_pose", self._make_bin_callback("green"), 10)
        self.create_subscription(PoseCommand, "bin_blue_pose", self._make_bin_callback("blue"), 10)
        self.create_subscription(PoseCommand, "bin_yellow_pose", self._make_bin_callback("yellow"), 10)

        self.get_logger().info("ObjectSortingNode inicializado.")
        self.get_logger().info(
            "Esperando:\n"
            "  - figure_type (std_msgs/String)\n"
            "  - pickup_pose (PoseCommand)\n"
            "  - bin_red_pose, bin_green_pose, bin_blue_pose, bin_yellow_pose (PoseCommand)\n"
            "Luego de tener todo, al recibir figure_type se ejecutará la rutina de clasificación."
        )

    # ------------------------------------------------------------------
    # Callbacks de suscripción
    # ------------------------------------------------------------------
    def figure_type_callback(self, msg: String) -> None:
        figure = msg.data.strip().lower()
        self.figure_type = figure
        self.get_logger().info(f"Tipo de figura recibido: '{figure}'")
        self.try_run_routine()

    def pickup_pose_callback(self, msg: PoseCommand) -> None:
        self.pickup_pose = msg
        self.get_logger().info(
            f"Pose de recolección recibida: "
            f"x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}"
        )

    def _make_bin_callback(self, color: str):
        def callback(msg: PoseCommand) -> None:
            self.bin_poses[color] = msg
            self.get_logger().info(
                f"Pose para caneca {color} recibida: "
                f"x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}"
            )

        return callback

    # ------------------------------------------------------------------
    # Lógica principal
    # ------------------------------------------------------------------
    def try_run_routine(self) -> None:
        """Verifica que toda la información esté disponible y ejecuta la rutina una vez."""
        if self.run_once:
            self.get_logger().warn("La rutina ya fue ejecutada una vez. Ignorando nueva petición.")
            return

        if self.figure_type is None:
            self.get_logger().warn("Aún no se ha recibido 'figure_type'.")
            return

        if self.pickup_pose is None:
            self.get_logger().warn("Aún no se ha recibido 'pickup_pose'.")
            return

        missing_bins = [c for c, pose in self.bin_poses.items() if pose is None]
        if missing_bins:
            self.get_logger().warn(
                f"Faltan poses para canecas: {', '.join(missing_bins)}. "
                "Publique en bin_<color>_pose antes de ejecutar la rutina."
            )
            return

        target_color = self.map_figure_to_color(self.figure_type)
        if target_color is None:
            self.get_logger().error(
                f"Tipo de figura desconocido: '{self.figure_type}'. "
                "Use: cubo, cilindro, pentagono, rectangulo."
            )
            return

        self.get_logger().info(
            f"Iniciando rutina de clasificación. Figura '{self.figure_type}' → caneca '{target_color}'."
        )
        try:
            self.execute_pick_and_place(target_color)
            self.run_once = True
            self.get_logger().info("Rutina de clasificación finalizada.")
        except Exception as exc:
            self.get_logger().error(f"Error ejecutando rutina: {exc}")

    @staticmethod
    def map_figure_to_color(figure: str) -> Optional[str]:
        """Mapea tipo de figura a color de caneca."""
        mapping = {
            "cubo": "red",
            "cube": "red",
            "cilindro": "green",
            "cylinder": "green",
            "pentagono": "blue",
            "pentágono": "blue",
            "pentagon": "blue",
            "rectangulo": "yellow",
            "rectángulo": "yellow",
            "rectangle": "yellow",
        }
        return mapping.get(figure)

    # ------------------------------------------------------------------
    # Ejecución de secuencia de pick & place
    # ------------------------------------------------------------------
    def execute_pick_and_place(self, target_color: str) -> None:
        if self.pickup_pose is None:
            raise RuntimeError("pickup_pose es None (no debería ocurrir aquí).")
        target_bin_pose = self.bin_poses[target_color]
        if target_bin_pose is None:
            raise RuntimeError(f"Pose de caneca '{target_color}' es None (no debería ocurrir aquí).")

        # Copias locales para no modificar los mensajes originales
        pickup = self.copy_pose(self.pickup_pose)
        bin_pose = self.copy_pose(target_bin_pose)

        # Poses "sobre" los puntos (mismo x,y pero z elevada)
        pickup_above = self.copy_pose(pickup)
        pickup_above.z += self.approach_offset_z

        bin_above = self.copy_pose(bin_pose)
        bin_above.z += self.approach_offset_z

        # Asegurar que cartesian_path sea coherente (se puede ajustar según pruebas)
        for pose in (pickup, pickup_above, bin_pose, bin_above):
            if pose.cartesian_path is None:
                pose.cartesian_path = False

        # 1) Asegurar gripper abierto
        self.set_gripper(open_gripper=True)
        # 2) Ir sobre la zona de recolección
        self.send_pose(pickup_above)
        # 3) Bajar a la zona de recolección
        self.send_pose(pickup)
        # 4) Cerrar gripper para agarrar el objeto
        self.set_gripper(open_gripper=False)
        # 5) Subir de nuevo (posición segura sobre la zona de recolección)
        self.send_pose(pickup_above)
        # 6) Ir sobre la caneca correspondiente
        self.send_pose(bin_above)
        # 7) Bajar hacia la caneca
        self.send_pose(bin_pose)
        # 8) Abrir gripper para soltar el objeto
        self.set_gripper(open_gripper=True)
        # 9) Subir a posición segura sobre la caneca
        self.send_pose(bin_above)

    # ------------------------------------------------------------------
    # Utilidades
    # ------------------------------------------------------------------
    @staticmethod
    def copy_pose(msg: PoseCommand) -> PoseCommand:
        new_msg = PoseCommand()
        new_msg.x = msg.x
        new_msg.y = msg.y
        new_msg.z = msg.z
        new_msg.roll = msg.roll
        new_msg.pitch = msg.pitch
        new_msg.yaw = msg.yaw
        new_msg.cartesian_path = msg.cartesian_path
        return new_msg

    def send_pose(self, pose: PoseCommand, delay_sec: float = 2.0) -> None:
        """Publica una pose en pose_command y espera un tiempo para que el movimiento se ejecute."""
        self.pose_pub.publish(pose)
        self.get_logger().info(
            f"Pose enviada: x={pose.x:.3f}, y={pose.y:.3f}, z={pose.z:.3f}, "
            f"roll={pose.roll:.3f}, pitch={pose.pitch:.3f}, yaw={pose.yaw:.3f}, "
            f"cartesian_path={pose.cartesian_path}"
        )
        # Espera sencilla para dar tiempo a MoveIt a planear y ejecutar.
        # Ajustar según la velocidad del robot y longitud de la trayectoria.
        time.sleep(delay_sec)

    def set_gripper(self, open_gripper: bool, delay_sec: float = 1.5) -> None:
        """Publica en open_gripper (true = abrir, false = cerrar)."""
        msg = Bool()
        msg.data = open_gripper
        self.gripper_pub.publish(msg)
        action = "ABRIR" if open_gripper else "CERRAR"
        self.get_logger().info(f"Comando gripper: {action}")
        time.sleep(delay_sec)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectSortingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

