# pincher_control/follow_joint_trajectory_node.py
#
# Este nodo conecta MoveIt con el hardware real del PhantomX Pincher usando
# el SDK de Dynamixel. Expone DOS servidores de acción FollowJointTrajectory:
#
#   1) /joint_trajectory_controller/follow_joint_trajectory  → grupo "arm" en MoveIt
#   2) /gripper_trajectory_controller/follow_joint_trajectory → grupo "gripper" en MoveIt
#
# La idea es:
#   - MoveIt calcula una trayectoria articular para el brazo o el gripper.
#   - Envía un goal FollowJointTrajectory al action server correspondiente.
#   - Este nodo convierte las posiciones (radianes) a ticks del AX-12A y
#     envía los comandos de posición a los servos reales.
#   - Además publica /joint_states con las posiciones COMANDADAS para que
#     robot_state_publisher y MoveIt conozcan el estado actual del robot.
#
# NOTA:
#   El gripper real se mueve con UN solo motor. En el URDF/SRDF hay dos joints
#   para las “uñitas” (finger1 y finger2), pero aquí ambas se mapean al MISMO
#   ID de servo. De esta forma:
#     phantomx_pincher_gripper_finger1_joint → ID_GRIPPER
#     phantomx_pincher_gripper_finger2_joint → ID_GRIPPER
#   En cada punto de trayectoria se envía un solo comando a ese servo, aunque
#   se actualicen las dos joints en /joint_states.

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

from dynamixel_sdk import PortHandler, PacketHandler

# Direcciones de registro en el AX-12A (igual que en control_servo.py)
ADDR_TORQUE_ENABLE    = 24
ADDR_GOAL_POSITION    = 30
ADDR_MOVING_SPEED     = 32
ADDR_TORQUE_LIMIT     = 34

# Rango básico de ticks en AX-12A (0–1023)
DXL_MIN_TICK = 0
DXL_MAX_TICK = 1023


class PincherFollowJointTrajectory(Node):
    """
    Nodo principal que implementa:
      - ActionServer FollowJointTrajectory para el brazo:
          /joint_trajectory_controller/follow_joint_trajectory
      - ActionServer FollowJointTrajectory para el gripper:
          /gripper_trajectory_controller/follow_joint_trajectory

    Ambos servers usan la MISMA lógica:
      * goal_callback → valida que las joints sean conocidas.
      * execute_callback → recorre los puntos de la trayectoria respetando
        time_from_start y manda las posiciones a los servos Dynamixel.

    Además:
      - Publica /joint_states con las posiciones COMANDADAS (brazo + gripper).
      - Convierte radianes del modelo de MoveIt a ticks (0–1023) de los AX-12A.
    """

    def __init__(self):
        super().__init__("pincher_follow_joint_trajectory")

        # -------------- Parámetros configurables del nodo --------------
        # Puerto serie donde está conectado el adaptador (U2D2, etc.)
        self.declare_parameter("port", "/dev/ttyUSB0")
        # Baudrate del bus Dynamixel
        self.declare_parameter("baudrate", 1000000)
        # Prefijo de joints según el URDF (por defecto phantomx_pincher_)
        self.declare_parameter("joint_prefix", "phantomx_pincher_")
        # Velocidad de movimiento (0–1023)
        self.declare_parameter("moving_speed", 200)
        # Límite de torque (0–1023)
        self.declare_parameter("torque_limit", 400)
        # ID del servo que mueve el gripper real
        self.declare_parameter("gripper_id", 5)

        port_name = self.get_parameter("port").get_parameter_value().string_value
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        prefix   = self.get_parameter("joint_prefix").get_parameter_value().string_value
        moving_speed = self.get_parameter("moving_speed").get_parameter_value().integer_value
        torque_limit = self.get_parameter("torque_limit").get_parameter_value().integer_value
        gripper_id   = self.get_parameter("gripper_id").get_parameter_value().integer_value

        # ------------ Mapa joint_name → ID de servo Dynamixel ------------
        #
        # IMPORTANTE: ajusta estos IDs a tu robot real.
        #
        # BRAZO:
        #   phantomx_pincher_arm_shoulder_pan_joint   → ID 1
        #   phantomx_pincher_arm_shoulder_lift_joint  → ID 2
        #   phantomx_pincher_arm_elbow_flex_joint     → ID 3
        #   phantomx_pincher_arm_wrist_flex_joint     → ID 4
        #
        # GRIPPER:
        #   phantomx_pincher_gripper_finger1_joint    → ID gripper_id
        #   phantomx_pincher_gripper_finger2_joint    → ID gripper_id (mismo servo)
        #
        # MoveIt enviará trayectorias separadas para el grupo arm y el grupo
        # gripper usando distintos controladores, pero aquí solo nos importa
        # el nombre de las joints.
        self.joint_to_id = {
            # Brazo (4 DOF)
            f"{prefix}arm_shoulder_pan_joint":   1,
            f"{prefix}arm_shoulder_lift_joint":  2,
            f"{prefix}arm_elbow_flex_joint":     3,
            f"{prefix}arm_wrist_flex_joint":     4,
            # Gripper (las dos fingers usan el mismo servo físico)
            f"{prefix}gripper_finger1_joint":    gripper_id,
            f"{prefix}gripper_finger2_joint":    gripper_id,
        }
        # Mismo convenio de signos que en control_servo.py:
        #   ID 1:  +1  (shoulder pan)
        #   ID 2:  -1  (shoulder lift)
        #   ID 3:  -1  (elbow flex)
        #   ID 4:  -1  (wrist flex)
        #   ID 5:  +1  (gripper)  → en nuestro caso gripper_id
        self.joint_sign = {
            1:  1,
            2: -1,
            3: -1,
            4: -1,
            gripper_id: 1,
        }

        self.get_logger().info(f"Mapa joint→ID: {self.joint_to_id}")
        self.get_logger().info(f"Signos por ID: {self.joint_sign}")

        # Lista de joints para /joint_states:
        #   - Todas las actuadas (keys de joint_to_id).
        #   - Más la joint “central” del gripper (gripper_joint) que es pasiva
        #     en este nodo pero existe en el URDF/SRDF y MoveIt la espera.
        self.prefix = prefix
        self.joint_state_names = list(self.joint_to_id.keys()) + [
            f"{prefix}gripper_joint",
        ]

        # Estado actual COMANDADO (en radianes) de cada joint.
        # Inicialmente asumimos 0.0 rad.
        self.current_positions = {name: 0.0 for name in self.joint_state_names}

        # Publisher de /joint_states + timer periódico (50 Hz)
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        self.joint_state_timer = self.create_timer(0.02, self.publish_joint_states)

        # -------------- Inicialización de comunicación Dynamixel --------------
        # Abre el puerto serie y configura el baudrate.
        self.port = PortHandler(port_name)
        if not self.port.openPort():
            self.get_logger().error(f"No se pudo abrir el puerto {port_name}")
            raise RuntimeError("Error abriendo el puerto Dynamixel")

        if not self.port.setBaudRate(baudrate):
            self.get_logger().error(f"No se pudo configurar el baudrate {baudrate}")
            raise RuntimeError("Error configurando el baudrate Dynamixel")

        # AX-12A usan protocolo 1.0
        self.packet = PacketHandler(1.0)

        # Configura cada servo: límite de torque, velocidad, torque enable.
        # Nota: el ID del gripper puede aparecer dos veces (finger1/finger2),
        # pero escribir los mismos valores dos veces no es un problema.
        for joint_name, dxl_id in self.joint_to_id.items():
            # Límite de torque
            self.packet.write2ByteTxRx(self.port, dxl_id, ADDR_TORQUE_LIMIT, torque_limit)
            # Velocidad
            self.packet.write2ByteTxRx(self.port, dxl_id, ADDR_MOVING_SPEED, moving_speed)
            # Habilitar torque
            self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 1)

        self.get_logger().info(
            f"Conectado a Dynamixel en {port_name} @ {baudrate} baud. "
            f"Velocidad={moving_speed}, Torque limit={torque_limit}, gripper_id={gripper_id}"
        )

        # -------------- Action Servers FollowJointTrajectory --------------
        # 1) Action server del BRAZO:
        #    Nombre del controlador en MoveIt:
        #      joint_trajectory_controller
        #    Action namespace:
        #      follow_joint_trajectory
        #    → nombre completo del action server:
        #      /joint_trajectory_controller/follow_joint_trajectory
        self._arm_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "joint_trajectory_controller/follow_joint_trajectory",
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
        )

        # 2) Action server del GRIPPER:
        #    Nombre del controlador en MoveIt:
        #      gripper_trajectory_controller
        #    Action namespace:
        #      follow_joint_trajectory
        #    → nombre completo del action server:
        #      /gripper_trajectory_controller/follow_joint_trajectory
        self._gripper_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "gripper_trajectory_controller/follow_joint_trajectory",
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
        )

        self.get_logger().info(
            "Action servers FollowJointTrajectory listos en:\n"
            "  - /joint_trajectory_controller/follow_joint_trajectory (brazo)\n"
            "  - /gripper_trajectory_controller/follow_joint_trajectory (gripper)"
        )

    # ======================================================================
    #   Callbacks del Action Server
    # ======================================================================

    def goal_callback(self, goal_request: FollowJointTrajectory.Goal) -> GoalResponse:
        """
        Se llama cuando llega un nuevo goal (tanto del brazo como del gripper).

        Aquí validamos:
          - que todas las joints de la trayectoria existen en joint_to_id,
          - que haya al menos un punto en la trayectoria.

        Si algo falla → GoalResponse.REJECT
        Si todo está bien → GoalResponse.ACCEPT
        """
        traj = goal_request.trajectory
        unknown_joints = [
            name for name in traj.joint_names if name not in self.joint_to_id
        ]

        if unknown_joints:
            self.get_logger().error(
                f"Goal rechazado: joints desconocidas: {unknown_joints}"
            )
            return GoalResponse.REJECT

        if not traj.points:
            self.get_logger().error("Goal rechazado: trayectoria vacía")
            return GoalResponse.REJECT

        self.get_logger().info(
            f"Nuevo goal recibido con joints {traj.joint_names} "
            f"y {len(traj.points)} puntos"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        """
        Se llama cuando el cliente (MoveIt u otro) solicita cancelar la trayectoria.

        Aquí aceptamos la cancelación y el execute_callback debe comprobar
        periódicamente goal_handle.is_cancel_requested para detener la ejecución.
        """
        self.get_logger().warn("Cancelación de trayectoria solicitada")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """
        Ejecuta la trayectoria punto a punto respetando los tiempos absolute:
          - goal_handle.request.trajectory.time_from_start

        Para cada punto:
          1. Espera hasta el instante adecuado (relativo al inicio del goal).
          2. Envía los comandos de posición a los servos correspondientes.
          3. Actualiza current_positions para que /joint_states refleje el cambio.

        Si se cancela el goal en mitad de la ejecución → marca el resultado
        como PATH_TOLERANCE_VIOLATED (puedes ajustar si prefieres otro código).
        """
        traj = goal_handle.request.trajectory
        joint_names = traj.joint_names

        # Marca de tiempo de inicio en reloj de pared (segundos)
        start_wall = time.time()

        # Recorremos cada punto de la trayectoria
        for i, point in enumerate(traj.points):
            # Comprobar cancelación
            if goal_handle.is_cancel_requested:
                self.get_logger().warn("Ejecución cancelada por el cliente")
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                return result

            # Instante absoluto en el que se debe ejecutar este punto
            t_point = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            target_wall = start_wall + t_point
            now = time.time()
            dt = target_wall - now
            if dt > 0.0:
                time.sleep(dt)

            # Enviar posiciones para todas las joints de este punto
            self.send_point(joint_names, point)

        # Si llegamos hasta aquí, consideramos que la trayectoria se completó bien
        self.get_logger().info("Trayectoria completada correctamente")
        goal_handle.succeed()

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    # ======================================================================
    #   Publicación de /joint_states
    # ======================================================================

    def publish_joint_states(self):
        """
        Publica periódicamente el estado COMANDADO de todas las joints
        (brazo + gripper) en el tópico /joint_states.

        NOTA:
          - Por simplicidad, usamos las posiciones que nosotros mismos mandamos
            a los servos (modelo "open-loop").
          - Si más adelante lees la posición real con dynamixel_sdk, aquí
            podrías publicar las posiciones reales en lugar de las comandadas.
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_state_names
        msg.position = [self.current_positions[name] for name in self.joint_state_names]

        self.joint_state_pub.publish(msg)

    # ======================================================================
    #   Utilidades internas: envío de puntos y conversión rad→tick
    # ======================================================================

    def send_point(self, joint_names, point: JointTrajectoryPoint):
        """
        Envía un único JointTrajectoryPoint a los servos.

        Parámetros:
          - joint_names: lista de nombres de joints en el orden de `point.positions`.
          - point.positions: posiciones deseadas en radianes.

        También actualiza self.current_positions para que /joint_states
        refleje las posiciones comandadas.

        Caso particular del gripper:
          - finger1 y finger2 se mapean al mismo servo (mismo ID).
          - Para evitar mandar dos veces el mismo comando al mismo servo en el
            mismo punto, guardamos qué IDs ya hemos comandado y solo escribimos
            una vez en hardware (pero actualizamos las posiciones de las dos
            joints en current_positions).
        """
        if len(point.positions) != len(joint_names):
            self.get_logger().error(
                "Punto de trayectoria con número de posiciones "
                "distinto al número de joints"
            )
            return

        # Lleva la cuenta de qué IDs ya han recibido comando en este punto
        commanded_ids = set()

        for j_name, pos_rad in zip(joint_names, point.positions):
            # Actualizar SIEMPRE la posición COMANDADA en current_positions
            # (en el sistema de coordenadas del URDF/MoveIt)
            if j_name in self.current_positions:
                self.current_positions[j_name] = pos_rad

            # Si la joint no está en joint_to_id, no tenemos servo para ella
            if j_name not in self.joint_to_id:
                self.get_logger().warn(
                    f"Joint {j_name} no conocida en joint_to_id, se ignora en hardware"
                )
                continue

            dxl_id = self.joint_to_id[j_name]

            # Evitar mandar dos veces al mismo servo (gripper finger1/finger2)
            if dxl_id in commanded_ids:
                continue
            commanded_ids.add(dxl_id)

            # 🔴 AQUÍ estaba el problema:
            # Usábamos pos_rad directamente, sin aplicar el signo de ese servo.
            #
            # Aplicar el signo para que el sentido físico del servo coincida
            # con el sentido de la articulación en el URDF/MoveIt.
            sign = self.joint_sign.get(dxl_id, 1)
            hw_rad = pos_rad * sign

            # Convertir radianes (lado hardware) a ticks del AX-12A
            tick = self.rad_to_dxl_tick(hw_rad)
            tick_clamped = max(DXL_MIN_TICK, min(DXL_MAX_TICK, tick))

            # Enviar comando de posición a ese servo
            _, err = self.packet.write2ByteTxRx(
                self.port, dxl_id, ADDR_GOAL_POSITION, tick_clamped
            )
            if err != 0:
                self.get_logger().warn(
                    f"Error al mandar posición a ID {dxl_id}: tick={tick_clamped}, "
                    f"err={err}"
                )

    def rad_to_dxl_tick(self, rad: float) -> int:
        """
        Convierte un ángulo en radianes (modelo de MoveIt/URDF) a ticks
        del AX-12A (0–1023).

        Suposición inicial:
          - 0 rad en el URDF ≈ centro mecánico del servo (~150°) ≈ 512 ticks.
          - Rango total ≈ 300° → ±150° alrededor del centro.

        Fórmula usada:
          tick = 512 + deg * (1023 / 300)

        Si al probar ves que el rango/dirección no coincide con lo que quieres
        para el gripper, se puede ajustar esta función o aplicar un offset solo
        para ese ID.
        """
        deg = math.degrees(rad)
        tick = 512.0 + deg * (1023.0 / 300.0)
        return int(round(tick))

    # ======================================================================
    #   Limpieza al destruir el nodo
    # ======================================================================

    def destroy_node(self):
        """
        Apaga el torque de todos los servos y cierra el puerto serie al
        destruir el nodo. También destruye explícitamente los ActionServers.
        """
        self.get_logger().info("Apagando torque y cerrando puerto Dynamixel...")
        try:
            for j_name, dxl_id in self.joint_to_id.items():
                self.packet.write1ByteTxRx(self.port, ADDR_TORQUE_ENABLE, 0)
        except Exception as e:
            self.get_logger().warn(f"Error al deshabilitar torque: {e}")

        try:
            self.port.closePort()
        except Exception as e:
            self.get_logger().warn(f"Error al cerrar puerto: {e}")

        # Destruir action servers explícitamente (buena práctica)
        try:
            self._arm_action_server.destroy()
            self._gripper_action_server.destroy()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    """
    Punto de entrada del nodo. Inicializa rclpy, crea el nodo y lo mantiene
    en ejecución hasta que se recibe Ctrl+C.
    """
    rclpy.init(args=args)
    node = PincherFollowJointTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
