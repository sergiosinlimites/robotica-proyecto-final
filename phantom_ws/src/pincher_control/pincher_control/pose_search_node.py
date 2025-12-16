#!/usr/bin/env python3

"""
Nodo auxiliar para buscar automáticamente una pose cercana que MoveIt
acepte como objetivo válido.

Dado una pose base (por ejemplo, la que te da `tf2_echo`), el nodo:

1. Publica esa pose como `PoseCommand` en `/pose_command`.
2. Escucha los logs de MoveIt en `/rosout` para ver si la planificación
   y ejecución han tenido éxito o han fallado ("Invalid goal state",
   "plan() failed or timeout reached").
3. Si falla, prueba pequeñas variaciones alrededor de la pose base en z
   y en pitch hasta encontrar una que funcione.

IMPORTANTE: Antes de lanzar este nodo, debe estar corriendo
`ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py ...`
para que existan `move_group`, `commander` y `follow_joint_trajectory`.
"""

import threading
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor

from rcl_interfaces.msg import Log
from phantomx_pincher_interfaces.msg import PoseCommand


class PoseSearchNode(Node):
    """Nodo que busca una pose cercana aceptable para MoveIt."""

    def __init__(self) -> None:
        super().__init__("pose_search_node")

        qos = QoSProfile(depth=100)
        self.pose_pub = self.create_publisher(PoseCommand, "/pose_command", 10)

        # Suscriptor a /rosout para leer logs de MoveIt / commander
        self.log_sub = self.create_subscription(Log, "/rosout", self.log_callback, qos)

        # Estado de la última planificación/ejecución observada en logs
        self._last_result: Optional[str] = None  # 'success' | 'fail' | None
        self._event = threading.Event()

        self.get_logger().info(
            "PoseSearchNode listo. Lanza 'phantomx_pincher_bringup' antes de usarlo."
        )

    # ------------------------------------------------------------------
    # Callbacks y utilidades
    # ------------------------------------------------------------------
    def log_callback(self, msg: Log) -> None:
        text = msg.msg

        # Mensajes típicos de fallo de planificación
        if "Invalid goal state" in text or "plan() failed or timeout reached" in text:
            self._last_result = "fail"
            self._event.set()
            self.get_logger().warn(f"Detectado fallo de planificación: {text}")
            return

        # Mensaje típico de éxito de ejecución en commander
        if "Execute request success!" in text:
            self._last_result = "success"
            self._event.set()
            self.get_logger().info("Detectado éxito de ejecución.")
            return

    def _build_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
    ) -> PoseCommand:
        msg = PoseCommand()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.roll = float(roll)
        msg.pitch = float(pitch)
        msg.yaw = float(yaw)
        msg.cartesian_path = False
        return msg

    def try_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        timeout_sec: float = 30.0,
    ) -> Optional[str]:
        """Publíca una pose y espera a que MoveIt reporte éxito o fallo.

        Devuelve 'success', 'fail' o None si hay timeout.
        """
        self._last_result = None
        self._event.clear()

        msg = self._build_pose(x, y, z, roll, pitch, yaw)
        self.get_logger().info(
            f"Probando pose: x={x:.3f}, y={y:.3f}, z={z:.3f}, "
            f"rpy=({roll:.3f}, {pitch:.3f}, {yaw:.3f})"
        )
        self.pose_pub.publish(msg)

        # Esperar hasta que llegue un log de éxito/fallo o se agote el timeout
        if not self._event.wait(timeout_sec):
            self.get_logger().warn("Timeout esperando respuesta de MoveIt.")
            return None

        return self._last_result

    # ------------------------------------------------------------------
    # Búsqueda local alrededor de una pose base
    # ------------------------------------------------------------------
    def search_around(
        self,
        base_pose: Tuple[float, float, float, float, float, float],
        dz_steps: Optional[List[float]] = None,
        d_pitch_steps: Optional[List[float]] = None,
    ) -> Optional[Tuple[float, float, float, float, float, float]]:
        """Prueba la pose base y variaciones pequeñas en z y pitch.

        base_pose: (x, y, z, roll, pitch, yaw)

        Devuelve la primera pose que planifica/ejecuta con éxito, o None.
        """
        x0, y0, z0, r0, p0, yaw0 = base_pose

        if dz_steps is None:
            dz_steps = [0.0, 0.01, -0.01, 0.02, -0.02, 0.03, -0.03]
        if d_pitch_steps is None:
            d_pitch_steps = [0.0, 0.1, -0.1, 0.2, -0.2]

        # Generar candidatos: combinaciones de dz y d_pitch
        candidates: List[Tuple[float, float, float, float, float, float]] = []
        for dz in dz_steps:
            for dp in d_pitch_steps:
                candidates.append((x0, y0, z0 + dz, r0, p0 + dp, yaw0))

        for (xc, yc, zc, rc, pc, ycaw) in candidates:
            res = self.try_pose(xc, yc, zc, rc, pc, ycaw, timeout_sec=12.0)
            if res == "success":
                self.get_logger().info(
                    "✅ Pose válida encontrada: "
                    f"x={xc:.3f}, y={yc:.3f}, z={zc:.3f}, "
                    f"rpy=({rc:.3f}, {pc:.3f}, {ycaw:.3f})"
                )
                return (xc, yc, zc, rc, pc, ycaw)
            elif res == "fail":
                self.get_logger().info("❌ Pose descartada (Invalid goal state / plan() failed).")
            else:
                self.get_logger().info("⏱️ Sin respuesta clara, probando siguiente candidato...")

        self.get_logger().error("No se encontró ninguna variación válida alrededor de la pose base.")
        return None


def main(args=None) -> None:
    import sys

    rclpy.init(args=args)
    node = PoseSearchNode()

    # Ejecutamos el ejecutor en un hilo aparte para que los callbacks de /rosout
    # sigan llegando mientras este hilo hace la búsqueda sin bloquear el spin.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Permitir pasar la pose base por línea de comandos:
    #   ros2 run pincher_control pose_search "0.081 -0.000 0.043 3.142 0.195 -0.000"
    base: Tuple[float, float, float, float, float, float]
    if len(sys.argv) >= 7:
        try:
            vals = [float(v) for v in sys.argv[1:7]]
            base = tuple(vals)  # type: ignore[assignment]
            node.get_logger().info(
                "Usando pose base desde argumentos de línea de comandos: "
                f"x={vals[0]:.3f}, y={vals[1]:.3f}, z={vals[2]:.3f}, "
                f"roll={vals[3]:.3f}, pitch={vals[4]:.3f}, yaw={vals[5]:.3f}"
            )
        except ValueError:
            node.get_logger().error(
                "Argumentos inválidos. Se esperaban 6 números: "
                "x y z roll pitch yaw. Usando pose base por defecto."
            )
            base = (0.1, 0.0, 0.140, 3.142, -0.005, 0.0)
    else:
        # Pose base por defecto (puedes cambiarla si quieres)
        base = (0.1, 0.0, 0.140, 3.142, -0.005, 0.0)

    try:
        valid_pose = node.search_around(base)
        if valid_pose is not None:
            x, y, z, r, p, yw = valid_pose
            node.get_logger().info(
                "=== Resultado final ===\n"
                f"x={x:.6f}, y={y:.6f}, z={z:.6f}, "
                f"roll={r:.6f}, pitch={p:.6f}, yaw={yw:.6f}"
            )
        else:
            node.get_logger().error("No se pudo encontrar una pose válida.")

    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
