import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


# ─── POSICIONES DE EJEMPLO ───────────────────────────────────────────────────
# Cada lista tiene 7 valores (uno por articulación A1..A7), en radianes.
# ~1 radián ≈ 57°, así que estos valores son movimientos moderados.

POSICION_A = [0.4, 0.0, 0.4, -0.5, 0.0,  0.4, 0.0]   # "lado izquierdo"
POSICION_B = [-0.4, 0.0, -0.4,  0.5, 0.0, -0.4, 0.0]  # "lado derecho"
POSICION_CERO = [0.0] * 7                               # posición de reposo


class MiClienteRobot(Node):
    """
    Nodo que mueve el KUKA iiwa7 de un lado a otro y vuelve al centro.
    Hereda de Node, que es la clase base de todos los nodos en ROS2.
    """

    def __init__(self):
        # Le damos un nombre al nodo. Así aparece en el grafo de ROS2.
        super().__init__(node_name="mi_cliente_robot")

        # Creamos el cliente de acción. Le decimos:
        #   - qué nodo somos (self)
        #   - qué tipo de acción usamos (FollowJointTrajectory)
        #   - el nombre del servidor al que nos conectamos
        self._cliente = ActionClient(
            node=self,
            action_type=FollowJointTrajectory,
            action_name="joint_trajectory_controller/follow_joint_trajectory",
        )

        # Esperamos a que el servidor esté listo. Sin esto el goal se perdería.
        self.get_logger().info("Esperando al servidor de acción...")
        while not self._cliente.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("  ...todavía esperando")
        self.get_logger().info("Servidor listo.")

    def mover_a(self, posiciones: list, segundos: int = 10):
        """
        Envía el robot a una posición concreta.

        posiciones : lista de 7 floats (radianes), uno por articulación
        segundos   : tiempo que tiene el robot para completar el movimiento
        """

        # Validación básica — mejor detectar el error aquí que en el robot
        if len(posiciones) != 7:
            self.get_logger().error(
                f"Se esperaban 7 posiciones, pero llegaron {len(posiciones)}."
            )
            return

        # ── Construimos el mensaje de trayectoria ──────────────────────────

        # El "goal" es el objetivo completo que mandamos al servidor
        goal = FollowJointTrajectory.Goal()

        # Tolerancia: si el robot llega con 1 segundo de retraso, aún lo aceptamos
        goal.goal_time_tolerance.sec = 1

        # Nombres de las articulaciones — el controlador los necesita para saber
        # a qué motor corresponde cada valor de posición
        for i in range(7):
            goal.trajectory.joint_names.append(f"lbr_A{i + 1}")

        # Un "point" es un instante de la trayectoria (aquí solo tenemos uno:
        # el destino final). El robot interpola solo el camino hasta él.
        punto = JointTrajectoryPoint()
        punto.positions = posiciones       # a dónde queremos llegar
        punto.velocities = [0.0] * 7      # velocidad 0 al llegar (parada suave)
        punto.time_from_start.sec = segundos  # en cuánto tiempo

        goal.trajectory.points.append(punto)

        # ── Enviamos el goal y esperamos respuesta ─────────────────────────

        self.get_logger().info(f"Enviando goal (tiempo: {segundos}s)...")
        futuro_goal = self._cliente.send_goal_async(goal)

        # spin_until_future_complete bloquea aquí hasta que el servidor
        # confirme si acepta o rechaza el goal
        rclpy.spin_until_future_complete(self, futuro_goal)
        handle = futuro_goal.result()

        if not handle.accepted:
            self.get_logger().error("El servidor rechazó el goal.")
            return

        self.get_logger().info("Goal aceptado — moviendo...")

        # Ahora esperamos a que el movimiento termine (o a que pase el tiempo límite)
        futuro_resultado = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            futuro_resultado,
            timeout_sec=segundos + 2,   # un poco más que el movimiento
        )

        # Comprobamos si el movimiento acabó bien
        codigo = futuro_resultado.result().result.error_code
        if codigo == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Movimiento completado con éxito.")
        else:
            self.get_logger().error(f"El movimiento falló (código: {codigo}).")


def main(args=None):
    rclpy.init(args=args)

    robot = MiClienteRobot()

    # ── Secuencia de movimientos ───────────────────────────────────────────
    robot.get_logger().info("=== Empezando secuencia ===")

    robot.get_logger().info("→ Moviendo a posición A")
    robot.mover_a(POSICION_A, segundos=8)

    robot.get_logger().info("→ Moviendo a posición B")
    robot.mover_a(POSICION_B, segundos=8)

    robot.get_logger().info("→ Volviendo al centro")
    robot.mover_a(POSICION_CERO, segundos=8)

    robot.get_logger().info("=== Secuencia completada ===")

    rclpy.shutdown()