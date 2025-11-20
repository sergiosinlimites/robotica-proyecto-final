# pincher_control/follow_joint_trajectory_node.py

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

DXL_MIN_TICK = 0
DXL_MAX_TICK = 1023


class PincherFollowJointTrajectory(Node):
    """
    Nodo que expone un Action Server FollowJointTrajectory para el grupo 'arm'
    del PhantomX Pincher y envía los comandos a los servos Dynamixel AX-12A.

    Este nodo está pensado para que MoveIt actúe como cliente de acción:
      - MoveIt planifica una trayectoria para el grupo 'arm'
      - Envía un goal FollowJointTrajectory al action server
      - Este nodo convierte las posiciones (radianes) a ticks y las manda
        a los servos en los tiempos indicados en time_from_start

    Además:
      - Publica /joint_states con las posiciones COMANDADAS (en radianes) para
        que robot_state_publisher y MoveIt puedan conocer el estado actual.

    NOTA SOBRE GRIPPER:
      - Más abajo hay líneas COMENTADAS para incluir el gripper:
          phantomx_pincher_gripper_finger1_joint
          phantomx_pincher_gripper_finger2_joint
      - Asumen que ambos dedos están acoplados al MISMO servo (ej. ID 5).
      - Cuando quieras usarlo, descomenta esas líneas y ajusta IDs si es necesario
        y asegúrate de que el controlador de MoveIt incluya también esas joints.
    """

    def __init__(self):
        super().__init__("pincher_follow_joint_trajectory")

        # ---------------- Parámetros configurables ----------------
        # Puerto serie donde está conectado el adaptador (U2D2, etc.)
        self.declare_parameter("port", "/dev/ttyUSB0")
        # Baudrate del bus Dynamixel
        self.declare_parameter("baudrate", 1000000)
        # Prefijo de joints (por defecto phantomx_pincher_)
        self.declare_parameter("joint_prefix", "phantomx_pincher_")
        # Velocidad de movimiento (0–1023)
        self.declare_parameter("moving_speed", 200)
        # Límite de torque (0–1023)
        self.declare_parameter("torque_limit", 800)

        port_name = self.get_parameter("port").get_parameter_value().string_value
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        prefix   = self.get_parameter("joint_prefix").get_parameter_value().string_value
        moving_speed = self.get_parameter("moving_speed").get_parameter_value().integer_value
        torque_limit = self.get_parameter("torque_limit").get_parameter_value().integer_value

        # ---------------- Mapa joint_name → ID de Dynamixel ----------------
        # AJUSTA ESTO según tus IDs reales en el robot físico.
        #
        # Los nombres de joints vienen del URDF / SRDF:
        #   phantomx_pincher_arm_shoulder_pan_joint
        #   phantomx_pincher_arm_shoulder_lift_joint
        #   phantomx_pincher_arm_elbow_flex_joint
        #   phantomx_pincher_arm_wrist_flex_joint
        #
        # Aquí asumimos:
        #   ID 1 → shoulder pan
        #   ID 2 → shoulder lift
        #   ID 3 → elbow flex
        #   ID 4 → wrist flex
        self.joint_to_id = {
            f"{prefix}arm_shoulder_pan_joint":   1,
            f"{prefix}arm_shoulder_lift_joint":  2,
            f"{prefix}arm_elbow_flex_joint":     3,
            f"{prefix}arm_wrist_flex_joint":     4,

            # ------------- GRIPPER (OPCIONAL – COMENTADO) -------------
            # Si tu gripper está controlado por un único servo (ej. ID 5),
            # y el URDF tiene dos joints (finger1 y finger2) unidas por mimic,
            # puedes mapear ambas al mismo ID de servo así:
            #
            # f"{prefix}gripper_finger1_joint": 5,
            # f"{prefix}gripper_finger2_joint": 5,
            #
            # ⚠️ Cuando actives esto:
            #   - Asegúrate de que MoveIt use un controlador que incluya estas
            #     joints en su lista (en el YAML de controladores).
            #   - Vigila que las trayectorias que lleguen tengan posiciones
            #     coherentes para ambas joints (normalmente serán simétricas).
        }

        self.get_logger().info(f"Mapa joint→ID: {self.joint_to_id}")

        # Lista fija de joints para publicar en /joint_states
        self.joint_names = list(self.joint_to_id.keys())

        # Estado actual COMANDADO (en radianes) de cada joint
        # Inicialmente asumimos 0.0 rad (equivalente a ~512 ticks en nuestro modelo)
        self.current_positions = {name: 0.0 for name in self.joint_names}

        # Publisher de /joint_states + timer periódico
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        # 50 Hz (0.02 s) es más que suficiente
        self.joint_state_timer = self.create_timer(0.02, self.publish_joint_states)

        # ---------------- Inicializar comunicación Dynamixel ----------------
        self.port = PortHandler(port_name)
        if not self.port.openPort():
            self.get_logger().error(f"No se pudo abrir el puerto {port_name}")
            raise RuntimeError("Error abriendo el puerto Dynamixel")

        if not self.port.setBaudRate(baudrate):
            self.get_logger().error(f"No se pudo configurar el baudrate {baudrate}")
            raise RuntimeError("Error configurando el baudrate Dynamixel")

        # AX-12A usan protocolo 1.0
        self.packet = PacketHandler(1.0)

        # Configurar cada servo: torque, velocidad, etc.
        for joint_name, dxl_id in self.joint_to_id.items():
            # Límite de torque
            self.packet.write2ByteTxRx(self.port, dxl_id, ADDR_TORQUE_LIMIT, torque_limit)
            # Velocidad
            self.packet.write2ByteTxRx(self.port, dxl_id, ADDR_MOVING_SPEED, moving_speed)
            # Habilitar torque
            self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 1)

        self.get_logger().info(
            f"Conectado a Dynamixel en {port_name} @ {baudrate} baud. "
            f"Velocidad={moving_speed}, Torque limit={torque_limit}"
        )

        # ---------------- Action Server FollowJointTrajectory ----------------
        # Nombre del action server: /joint_trajectory_controller/follow_joint_trajectory
        # Debe coincidir con la configuración de controladores de MoveIt.
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "joint_trajectory_controller/follow_joint_trajectory",
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
        )

        self.get_logger().info(
            "Action server FollowJointTrajectory listo en "
            "'/joint_trajectory_controller/follow_joint_trajectory'"
        )

    # ----------------------------------------------------------------------
    # Callbacks del Action Server
    # ----------------------------------------------------------------------
    def goal_callback(self, goal_request: FollowJointTrajectory.Goal) -> GoalResponse:
        """
        Se llama cuando llega un nuevo goal. Aquí comprobamos que las joints
        que pide MoveIt son conocidas y que podemos controlarlas.
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
        Aquí podríamos implementar stop de emergencia. Por ahora solo aceptamos
        la cancelación; el bucle de ejecución debe respetarlo y dejar de enviar
        nuevos comandos si goal_handle.is_cancel_requested es True.
        """
        self.get_logger().warn("Cancelación de trayectoria solicitada")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """
        Ejecuta la trayectoria de joints punto a punto respetando los tiempos
        time_from_start. Convierte radianes → ticks y manda GOAL_POSITION a
        cada Dynamixel correspondiente.
        """
        traj = goal_handle.request.trajectory
        joint_names = traj.joint_names

        # Marca de tiempo de inicio en reloj de pared (segundos)
        start_wall = time.time()

        # Recorremos cada punto de la trayectoria
        for i, point in enumerate(traj.points):
            if goal_handle.is_cancel_requested:
                self.get_logger().warn("Ejecución cancelada por el cliente")
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                return result

            # Calcular instante absoluto en el que se debe ejecutar este punto
            t_point = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            target_wall = start_wall + t_point
            now = time.time()
            dt = target_wall - now
            if dt > 0.0:
                time.sleep(dt)

            # Enviar posiciones para todas las joints de este punto
            self.send_point(joint_names, point)

        # Si llegamos hasta aquí, consideramos la trayectoria ejecutada
        self.get_logger().info("Trayectoria completada correctamente")
        goal_handle.succeed()

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    # ----------------------------------------------------------------------
    # Publicación de /joint_states
    # ----------------------------------------------------------------------
    def publish_joint_states(self):
        """
        Publica las posiciones COMANDADAS de las joints en /joint_states.

        Nota: por simplicidad usamos las posiciones que le hemos mandado
        a los servos (modelo "open-loop"). Si más adelante lees feedback
        real de los AX-12A, aquí podrías publicar las posiciones medidas.
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [self.current_positions[name] for name in self.joint_names]

        self.joint_state_pub.publish(msg)

    # ----------------------------------------------------------------------
    # Utilidades internas
    # ----------------------------------------------------------------------
    def send_point(self, joint_names, point: JointTrajectoryPoint):
        """
        Envía un JointTrajectoryPoint a los servos:
          - joint_names define el orden
          - point.positions está en radianes

        Además actualiza self.current_positions para que /joint_states
        refleje las posiciones comandadas.
        """
        if len(point.positions) != len(joint_names):
            self.get_logger().error(
                "Punto de trayectoria con número de posiciones "
                "distinto al número de joints"
            )
            return

        for j_name, pos_rad in zip(joint_names, point.positions):
            if j_name not in self.joint_to_id:
                self.get_logger().warn(
                    f"Joint {j_name} no conocida, se ignora en este punto"
                )
                continue

            dxl_id = self.joint_to_id[j_name]
            tick = self.rad_to_dxl_tick(pos_rad)
            tick_clamped = max(DXL_MIN_TICK, min(DXL_MAX_TICK, tick))

            # Enviar comando de posición al servo
            _, err = self.packet.write2ByteTxRx(
                self.port, dxl_id, ADDR_GOAL_POSITION, tick_clamped
            )
            if err != 0:
                self.get_logger().warn(
                    f"Error al mandar posición a ID {dxl_id}: tick={tick_clamped}, "
                    f"err={err}"
                )

            # Actualizar posición comandada para /joint_states
            self.current_positions[j_name] = pos_rad

    def rad_to_dxl_tick(self, rad: float) -> int:
        """
        Convierte un ángulo en radianes (modelo de MoveIt/URDF) a ticks
        del AX-12A (0–1023).

        Suposición inicial:
          - 0 rad en URDF = servo centrado (~150°) = 512 ticks
          - Rango útil ≈ ±150° → 0–300°

        Fórmula:
          tick = 512 + deg * (1023 / 300)
        """
        deg = math.degrees(rad)
        tick = 512.0 + deg * (1023.0 / 300.0)
        return int(round(tick))

    def destroy_node(self):
        """
        Apaga torque y cierra el puerto al destruir el nodo.
        """
        self.get_logger().info("Apagando torque y cerrando puerto Dynamixel...")
        try:
            for j_name, dxl_id in self.joint_to_id.items():
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 0)
        except Exception as e:
            self.get_logger().warn(f"Error al deshabilitar torque: {e}")

        try:
            self.port.closePort()
        except Exception as e:
            self.get_logger().warn(f"Error al cerrar puerto: {e}")

        super().destroy_node()


def main(args=None):
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
