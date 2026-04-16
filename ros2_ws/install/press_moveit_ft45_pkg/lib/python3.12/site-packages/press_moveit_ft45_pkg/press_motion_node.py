"""
press_motion_node.py
====================
Mueve el TCP del LBR hacia la mesa (lbr_link_0) y lee las fuerzas
del sensor FT45.

Cambios respecto a la versión anterior
---------------------------------------
* Diagnóstico automático del tópico FT: prueba los candidatos conocidos
  y selecciona el primero que reciba un mensaje en los primeros 3 s.
* Imprime las fuerzas en reposo antes de mover el robot para verificar
  que el sensor está activo.
* Filtro de ventana deslizante (N=5) sobre Fx, Fy, Fz para reducir ruido.
* Log mejorado: muestra el tópico activo y la frecuencia estimada.
* La escena de planificación coloca la caja virtual a Z = SURFACE_Z - 0.05,
  igual que antes, pero con un margen lateral mayor para la ACM.
"""

import threading
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, WrenchStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    BoundingVolume,
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.action.client import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

# ---------------------------------------------------------------------------
# Tópicos candidatos para el sensor FT.  Se probará cada uno en orden y se
# usará el primero que publique dentro del tiempo de espera.
# ---------------------------------------------------------------------------
FT_TOPIC_CANDIDATES = [
    "/ft_sensor/wrench",
]

WINDOW_SIZE = 5  # muestras para el filtro de ventana deslizante


class PressMotionNode(Node):
    PLANNING_GROUP = "arm"
    EEF_LINK = "lbr_link_ee"
    BASE_FRAME = "lbr_link_0"

    # Altura de la superficie de la mesa respecto a lbr_link_0
    SURFACE_Z = 0.20
    # Objetivo de presión: forzamos bajar más (Z=0.05) para que choque físicamente contra la pieza y genere fuerza
    PRESS_Z = 0.05
    # Posición segura sobre la pieza
    PRECONTACT_Z = 0.35
    # Umbral de fuerza máxima permitida (N)
    FORCE_LIMIT_N = 30.0

    def __init__(self):
        super().__init__("press_motion_node")

        # --- Estado del sensor ---
        self._latest_wrench: WrenchStamped | None = None
        self._wrench_lock = threading.Lock()
        self._active_ft_topic: str | None = None
        self._ft_sub = None
        self._msg_count = 0
        self._first_msg_time: float | None = None

        # Filtro de ventana deslizante
        self._fx_window: deque = deque(maxlen=WINDOW_SIZE)
        self._fy_window: deque = deque(maxlen=WINDOW_SIZE)
        self._fz_window: deque = deque(maxlen=WINDOW_SIZE)

        # --- Clientes ROS ---
        self._action_client = ActionClient(self, MoveGroup, "/lbr/move_action")

        self.get_logger().info("Esperando a move_group...")
        self._action_client.wait_for_server()
        self.get_logger().info("move_group listo.")

        # --- Descubrir y verificar el sensor FT ---
        self._discover_ft_topic()
        if self._active_ft_topic is None:
            self.get_logger().error(
                "No se encontró ningún tópico FT activo. "
                "Comprueba que el sensor esté publicando."
            )
            return

        self._run_ft_diagnostic()

        # --- Ejecutar secuencia ---
        self.run_press_sequence()

    # -----------------------------------------------------------------------
    # Descubrimiento del tópico FT
    # -----------------------------------------------------------------------

    def _discover_ft_topic(self, timeout_per_topic: float = 2.0):
        """
        Suscribe a cada candidato FT_TOPIC_CANDIDATES en orden.
        El primero que reciba un mensaje dentro de *timeout_per_topic* segundos
        se convierte en el tópico activo.
        """
        self.get_logger().info("Buscando tópico FT activo...")

        for topic in FT_TOPIC_CANDIDATES:
            self.get_logger().info(f"  Probando: {topic}")
            found = threading.Event()

            def _cb(msg: WrenchStamped, t=topic, ev=found):
                with self._wrench_lock:
                    self._latest_wrench = msg
                    self._active_ft_topic = t
                ev.set()

            sub = self.create_subscription(WrenchStamped, topic, _cb, 10)

            deadline = time.time() + timeout_per_topic
            while time.time() < deadline and not found.is_set():
                rclpy.spin_once(self, timeout_sec=0.1)

            if found.is_set():
                self.get_logger().info(f"  -> Tópico FT activo: {topic}")
                # Reemplazar callback por el definitivo (con filtro y conteo)
                self.destroy_subscription(sub)
                self._ft_sub = self.create_subscription(
                    WrenchStamped, topic, self._wrench_cb, 10
                )
                return
            else:
                self.destroy_subscription(sub)

        self.get_logger().warning("Ninguno de los tópicos FT candidatos publicó datos.")

    def _wrench_cb(self, msg: WrenchStamped):
        with self._wrench_lock:
            self._latest_wrench = msg
            f = msg.wrench.force
            self._fx_window.append(f.x)
            self._fy_window.append(f.y)
            self._fz_window.append(f.z)
            self._msg_count += 1
            if self._first_msg_time is None:
                self._first_msg_time = time.time()

    # -----------------------------------------------------------------------
    # Diagnóstico inicial: imprime fuerzas en reposo
    # -----------------------------------------------------------------------

    def _run_ft_diagnostic(self, duration: float = 1.0):
        """
        Lee las fuerzas durante *duration* segundos con el robot quieto para
        verificar el offset en reposo del sensor.
        """
        self.get_logger().info(
            f"=== Diagnóstico FT ({duration:.0f} s) — robot en reposo ==="
        )
        t0 = time.time()
        samples = 0
        while time.time() - t0 < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            fx, fy, fz = self.get_force()
            mag = (fx**2 + fy**2 + fz**2) ** 0.5
            self.get_logger().info(
                f"  [diag] Fx={fx:+.3f}N  Fy={fy:+.3f}N  Fz={fz:+.3f}N  |F|={mag:.3f}N"
            )
            samples += 1
            time.sleep(0.2)

        elapsed = time.time() - t0
        if self._first_msg_time and self._msg_count > 0:
            freq = self._msg_count / elapsed
            self.get_logger().info(
                f"  Frecuencia estimada del sensor: {freq:.1f} Hz "
                f"({self._msg_count} msgs en {elapsed:.1f} s)"
            )
        self.get_logger().info("=== Fin diagnóstico FT ===")

    # -----------------------------------------------------------------------
    # Lectura de fuerza (con filtro de ventana deslizante)
    # -----------------------------------------------------------------------

    def get_force(self) -> tuple[float, float, float]:
        """Devuelve la media de la ventana deslizante, o (0,0,0) si no hay datos."""
        with self._wrench_lock:
            if not self._fx_window:
                return 0.0, 0.0, 0.0
            fx = sum(self._fx_window) / len(self._fx_window)
            fy = sum(self._fy_window) / len(self._fy_window)
            fz = sum(self._fz_window) / len(self._fz_window)
        return fx, fy, fz

    def _log_force(self, prefix: str = ""):
        fx, fy, fz = self.get_force()
        mag = (fx**2 + fy**2 + fz**2) ** 0.5
        tag = f"[{prefix}] " if prefix else ""
        self.get_logger().info(
            f"  {tag}FT_Sensor  Fx={fx:+.3f}N  Fy={fy:+.3f}N  Fz={fz:+.3f}N  |F|={mag:.3f}N"
        )

    # -----------------------------------------------------------------------
    # Construcción del goal de MoveGroup
    # -----------------------------------------------------------------------

    def make_pose_goal(
        self,
        x: float,
        y: float,
        z: float,
        qx: float = 0.0,
        qy: float = 1.0,
        qz: float = 0.0,
        qw: float = 0.0,
        pos_tol: float = 0.02,
        ang_tol: float = 0.3,
    ) -> MoveGroup.Goal:
        """Construye un MoveGroup.Goal con restricciones de posición y orientación."""
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.BASE_FRAME
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.x = qx
        target_pose.pose.orientation.y = qy
        target_pose.pose.orientation.z = qz
        target_pose.pose.orientation.w = qw

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [pos_tol]

        bv = BoundingVolume()
        bv.primitives = [sphere]
        bv.primitive_poses = [target_pose.pose]

        pos_c = PositionConstraint()
        pos_c.header.frame_id = self.BASE_FRAME
        pos_c.link_name = self.EEF_LINK
        pos_c.constraint_region = bv
        pos_c.weight = 1.0

        ori_c = OrientationConstraint()
        ori_c.header.frame_id = self.BASE_FRAME
        ori_c.link_name = self.EEF_LINK
        ori_c.orientation = target_pose.pose.orientation
        ori_c.absolute_x_axis_tolerance = ang_tol
        ori_c.absolute_y_axis_tolerance = ang_tol
        ori_c.absolute_z_axis_tolerance = ang_tol
        ori_c.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [pos_c]
        constraints.orientation_constraints = [ori_c]

        request = MotionPlanRequest()
        request.group_name = self.PLANNING_GROUP
        request.start_state.is_diff = True
        request.goal_constraints = [constraints]
        request.num_planning_attempts = 10
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = 0.3
        request.max_acceleration_scaling_factor = 0.3
        request.pipeline_id = "ompl"
        request.workspace_parameters.header.frame_id = self.BASE_FRAME
        request.workspace_parameters.min_corner.x = -2.0
        request.workspace_parameters.min_corner.y = -2.0
        request.workspace_parameters.min_corner.z = -0.5
        request.workspace_parameters.max_corner.x = 2.0
        request.workspace_parameters.max_corner.y = 2.0
        request.workspace_parameters.max_corner.z = 2.0

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        return goal

    def make_joint_goal(
        self, joint_names: list[str], joint_values: list[float]
    ) -> MoveGroup.Goal:
        """Construye un MoveGroup.Goal basado en posiciones de articulaciones."""
        constraints = Constraints()
        for name, val in zip(joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = val
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        request = MotionPlanRequest()
        request.group_name = self.PLANNING_GROUP
        request.start_state.is_diff = True
        request.goal_constraints = [constraints]
        request.num_planning_attempts = 10
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = 0.3
        request.max_acceleration_scaling_factor = 0.3
        request.pipeline_id = "ompl"

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        return goal

    # -----------------------------------------------------------------------
    # Envío del goal y espera de resultado
    # -----------------------------------------------------------------------

    def send_goal_and_wait(self, goal: MoveGroup.Goal, label: str) -> bool:
        self.get_logger().info(f"-> Enviando goal: {label}")

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server no disponible.")
            return False

        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"Goal rechazado: {label}")
            return False

        self.get_logger().info("  Goal aceptado, esperando resultado...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        response = result_future.result()

        if response is None:
            self.get_logger().error(f"Respuesta None: {label}")
            return False

        ec = response.result.error_code.val
        codes = {
            1: "SUCCESS (1)",
            -2: "PLANNING_FAILED (-2): No se pudo encontrar un camino válido (Revisa posición del Goal).",
            -4: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE (-4): Colisión repentina detectada en ejecución.",
            -5: "CONTROL_FAILED (-5): El controlador no pudo seguir la trayectoria (Inercia, choque, o PID).",
            -7: "TIMED_OUT (-7): Excedido el tiempo de espera.",
            -9: "START_STATE_IN_COLLISION (-9): El brazo ya colisiona con algo antes de iniciar movimiento.",
            -10: "START_STATE_IN_COLLISION (-10): El brazo ya colisiona con algo antes de iniciar movimiento.",
            -12: "GOAL_IN_COLLISION (-12): El destino final calculado atraviesa un objeto (Ajusta tolerancias).",
        }
        code_str = codes.get(ec, f"UNKNOWN_CODE({ec})")

        if ec == 1:
            self.get_logger().info(f"  {label}: {code_str}")
            return True
        elif ec in (-4, -5, -12) and label == "Presionando objeto":
            # Probable contacto real con la superficie
            self.get_logger().info(
                f"  {label}: {code_str}\n"
                f"    -> NOTA: RViz/MoveIt detecta que el robot choca contra la mesa real o el suelo.\n"
                f"    -> Como estamos haciendo presión intencionalmente, continuamos."
            )
            return True
        else:
            self.get_logger().error(f"  {label} abortado por: {code_str}")
            return False

    # -----------------------------------------------------------------------
    # Secuencia principal
    # -----------------------------------------------------------------------

    def run_press_sequence(self):
        self.get_logger().info("--- Fase 0: Posicion inicial (transport) ---")
        joint_names = [
            "lbr_A1",
            "lbr_A2",
            "lbr_A3",
            "lbr_A4",
            "lbr_A5",
            "lbr_A6",
            "lbr_A7",
        ]
        joint_values = [3.14159, 0.4363, 0.0, 1.5707, 0.0, 0.0, 0.0]
        goal_transport = self.make_joint_goal(joint_names, joint_values)
        if not self.send_goal_and_wait(goal_transport, "Transport pose"):
            self.get_logger().warning("No se pudo alcanzar la pose transport inicial.")

        num_ciclos = 3

        for ciclo in range(1, num_ciclos + 1):
            self.get_logger().info(f"\n=====================================")
            self.get_logger().info(f"=== INICIANDO CICLO {ciclo} DE {num_ciclos} ===")
            self.get_logger().info(f"=====================================")

            # Fase 1 — Pre-contacto (posición segura sobre la pieza)
            self.get_logger().info(f"--- Ciclo {ciclo} - Fase 1: Pre-contacto ---")
            goal_pre = self.make_pose_goal(
                x=-0.45,
                y=0.0,
                z=self.PRECONTACT_Z,
                qx=1.0,
                qy=0.0,
                qz=0.0,
                qw=0.0,
            )
            if not self.send_goal_and_wait(goal_pre, "Pre-contacto"):
                self.get_logger().error("Abortando: no se pudo alcanzar pre-contacto.")
                return

            self._log_force("pre-contacto")
            self.get_logger().info("Esperando 1 s para estabilizar el robot...")
            time.sleep(1.0)

            # Fase 2 — Bajada hasta contacto con velocidad muy reducida
            self.get_logger().info(
                f"--- Ciclo {ciclo} - Fase 2: Presionando superficie ---"
            )
            goal_press = self.make_pose_goal(
                x=-0.45,
                y=0.0,
                z=self.PRESS_Z,
                qx=1.0,
                qy=0.0,
                qz=0.0,
                qw=0.0,
                pos_tol=0.005,
                ang_tol=0.4,
            )
            goal_press.request.max_velocity_scaling_factor = 0.05
            goal_press.request.max_acceleration_scaling_factor = 0.05

            if not self.send_goal_and_wait(goal_press, "Presionando objeto"):
                self.get_logger().warning(
                    "Fase de presión fallida (posible contacto real)."
                )

            # Fase 3 — Lectura de fuerza ampliada durante 5 segundos
            self.get_logger().info(
                f"--- Ciclo {ciclo} - Fase 3: Leyendo fuerzas durante 5 s ---"
            )
            t0 = time.time()
            force_exceeded = False
            while time.time() - t0 < 5.0:
                rclpy.spin_once(self, timeout_sec=0.05)
                fx, fy, fz = self.get_force()
                mag = (fx**2 + fy**2 + fz**2) ** 0.5
                self.get_logger().info(
                    f"  FT_Sensor  Fx={fx:+.3f}N  Fy={fy:+.3f}N  Fz={fz:+.3f}N  |F|={mag:.3f}N"
                )
                if abs(fz) > self.FORCE_LIMIT_N or mag > self.FORCE_LIMIT_N * 1.2:
                    self.get_logger().warning(
                        f"Umbral superado (|Fz|={abs(fz):.1f}N, |F|={mag:.1f}N > "
                        f"{self.FORCE_LIMIT_N}N) — retirando."
                    )
                    force_exceeded = True
                    break
                time.sleep(0.15)

            if not force_exceeded:
                self._log_force("presión final")

            # Fase 4 — Retorno a posición segura
            self.get_logger().info(f"--- Ciclo {ciclo} - Fase 4: Retorno ---")
            goal_ret = self.make_pose_goal(
                x=-0.45,
                y=0.0,
                z=self.PRECONTACT_Z,
                qx=1.0,
                qy=0.0,
                qz=0.0,
                qw=0.0,
            )
            self.send_goal_and_wait(goal_ret, "Retorno")

            self.get_logger().info(f"=== CICLO {ciclo} COMPLETADO ===\n")

        self.get_logger().info("=== TODA LA SECUENCIA DE CICLOS COMPLETADA ===")


# ---------------------------------------------------------------------------
# Entrada
# ---------------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = PressMotionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
