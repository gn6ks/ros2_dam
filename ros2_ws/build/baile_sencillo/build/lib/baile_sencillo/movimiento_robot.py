import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


# Posiciones en radianes por articulación (A1..A7). ~1 rad ≈ 57°
CERO    = [0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0]
BAILE_1 = [-1.0,  0.8, -0.8,  1.0,  1.0,  0.8,  1.0]
BAILE_2 = [ 1.0, -0.8,  0.8, -1.0, -1.0, -0.8, -1.0]

SECUENCIA_BAILE = [
    (BAILE_1, "-- izquierda"),
    (BAILE_2, "-- derecha"),
    (BAILE_1, "-- izquierda"),
    (BAILE_2, "-- derecha"),
    (BAILE_1, "-- izquierda"),
    (BAILE_2, "-- derecha"),
    (CERO,    "-- reposo"),
]


class ClienteRobotIiwa7(Node):
    """
    Nodo ROS2 que controla el KUKA iiwa7 mediante la accion
    FollowJointTrajectory. Envia goals de posicion secuenciales
    y espera confirmacion de cada uno antes de continuar

    param: nodo con el que hereda absolutamente todo
    """

    def __init__(self):
        super().__init__(node_name="iiwa7_baile")
        self._cliente = ActionClient(
            node=self,
            action_type=FollowJointTrajectory,
            action_name="joint_trajectory_controller/follow_joint_trajectory",
        )
        self.get_logger().info("waiting server action...")
        while not self._cliente.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("still waiting...")
        self.get_logger().info("server [OK]")

    def mover_a(self, posiciones: list, segundos: int = 5) -> bool:
        """
        Envia el robot a una posicion y bloquea hasta que termina.

        :param posiciones: 7 floats en radianes, uno por articulacion A1-A7
        :param segundos:   tiempo maximo para completar el movimiento
        :returns:          true [OK] movimiento false [X] movimiento 
        """
        if len(posiciones) != 7:
            self.get_logger().error(f"7 positions required, only took {len(posiciones)}")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.goal_time_tolerance.sec = 1

        for i in range(7):
            goal.trajectory.joint_names.append(f"lbr_A{i + 1}")

        punto = JointTrajectoryPoint()
        punto.positions  = posiciones
        punto.velocities = [0.0] * 7
        punto.time_from_start.sec = segundos
        goal.trajectory.points.append(punto)

        self.get_logger().info(f"sending goal (t: {segundos}s)...")
        futuro_goal = self._cliente.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, futuro_goal)

        handle = futuro_goal.result()
        if not handle.accepted:
            self.get_logger().error("goal rejected [X]")
            return False
        self.get_logger().info("goal accepted [OK]")

        futuro_resultado = handle.get_result_async()
        rclpy.spin_until_future_complete(self, futuro_resultado, timeout_sec=segundos + 2)

        exito = futuro_resultado.result().result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        if exito:
            self.get_logger().info("completion [OK]")
        else:
            self.get_logger().error(f"movement failed (code: {futuro_resultado.result().result.error_code})")
        return exito

    def ejecutar_secuencia(self, secuencia: list, segundos_por_paso: int = 5):
        """
        Ejecuta una lista de posiciones en orden

        :param secuencia:         lista de objects¿? (posiciones, etiqueta)
        :param segundos_por_paso: tiempo por movimiento, aplicado a todos los pasos
        """
        self.get_logger().info(f"-- INITIAL SECUENCE ({len(secuencia)} steps")
        for i, (posicion, etiqueta) in enumerate(secuencia):
            self.get_logger().info(f"[{i + 1}/{len(secuencia)}] {etiqueta}")
            self.mover_a(posicion, segundos_por_paso)
        self.get_logger().info("-- END SECUENCE")


def main(args=None):
    rclpy.init(args=args)
    robot = ClienteRobotIiwa7()
    robot.ejecutar_secuencia(SECUENCIA_BAILE, segundos_por_paso=5)
    rclpy.shutdown()