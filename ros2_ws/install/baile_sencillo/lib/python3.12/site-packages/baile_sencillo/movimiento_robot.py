import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

# Cada lista tiene 7 valores (uno por articulación A1..A7), en radianes.
# ~1 radián ≈ 57°, así que estos valores son movimientos moderados.

POSICION_A = [0.6, 0.0, 0.6, -0.7, 0.0,  0.8, 0.0]   # "lado izquierdo"
POSICION_B = [-0.7, 0.0, -0.3,  0.2, 0.7, -0.4, 0.0]  # "lado derecho"
POSICION_CERO = [0.0] * 7                               # posición de reposo


class cliente_robot_iiwa7(Node):

    def __init__(self):
        super().__init__(node_name="iiwa7_roboto")

        self._cliente = ActionClient(
            node=self,
            action_type=FollowJointTrajectory,
            action_name="joint_trajectory_controller/follow_joint_trajectory",
        )

        self.get_logger().info("waiting server action")
        while not self._cliente.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("waiting")
        self.get_logger().info("server [OK]")

    def mover_a(self, posiciones: list, segundos: int = 10):
        #Validacion basica
        if len(posiciones) != 7:
            self.get_logger().error(
                f"Se esperaban 7 posiciones, pero llegaron {len(posiciones)}."
            )
            return

        goal = FollowJointTrajectory.Goal()

        # tiempo de toleracion (1s), para el retraso
        goal.goal_time_tolerance.sec = 1

        # motor = valor posicion
        for i in range(7):
            goal.trajectory.joint_names.append(f"lbr_A{i + 1}")

        punto = JointTrajectoryPoint()
        punto.positions = posiciones       # a donde queremos llegar
        punto.velocities = [0.0] * 7      # velocidad 0 al llegar (parada suave)
        punto.time_from_start.sec = segundos  # en cuanto tiempo

        goal.trajectory.points.append(punto)

        self.get_logger().info(f"sending goal (t: {segundos}s)...")
        futuro_goal = self._cliente.send_goal_async(goal)

        # acepta / rechaza el goal
        rclpy.spin_until_future_complete(self, futuro_goal)
        handle = futuro_goal.result()

        if not handle.accepted:
            self.get_logger().error("server rejected goal [X]")
            return

        self.get_logger().info("goal acepted [OK]")

        futuro_resultado = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            futuro_resultado,
            timeout_sec=segundos + 2,
        )

        # comprobamos si todo OK al acabar
        codigo = futuro_resultado.result().result.error_code
        if codigo == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("completion [OK]")
        else:
            self.get_logger().error(f"moviment failed (code: {codigo}).")


def main(args=None):
    rclpy.init(args=args)

    robot = cliente_robot_iiwa7()

    robot.get_logger().info("-- INITIAL SECUENCE")

    robot.get_logger().info("-> to A")
    robot.mover_a(POSICION_A, segundos=5)

    robot.get_logger().info("-> to B")
    robot.mover_a(POSICION_B, segundos=5)

    robot.get_logger().info("-> to REST POSITION")
    robot.mover_a(POSICION_CERO, segundos=5)

    robot.get_logger().info("-- END OF SECUENCE")

    rclpy.shutdown()