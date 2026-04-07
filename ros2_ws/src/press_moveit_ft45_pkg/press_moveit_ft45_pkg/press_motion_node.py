import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    AllowedCollisionMatrix,
    AllowedCollisionEntry,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from geometry_msgs.msg import PoseStamped, Pose, Point, Vector3
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import WrenchStamped
import threading
import time


class PressMotionNode(Node):

    PLANNING_GROUP = "arm"
    EEF_LINK      = "tool_tcp_link"
    BASE_FRAME    = "lbr_link_0"

    # Plataforma sobre la que va a presionar (en coordenadas de BASE_FRAME)
    SURFACE_Z     = 0.45   # altura superior de la caja en metros
    PRESS_Z       = 0.44   # cuánto baja (ligero overlap → Gazebo genera contacto)
    PRECONTACT_Z  = 0.55   # posición segura sobre la superficie
    FORCE_LIMIT_N = 30.0   # para la secuencia si supera este umbral

    def __init__(self):
        super().__init__('press_motion_node')

        # subscripcion sensor de fuerza
        self._latest_wrench = None
        self._wrench_lock   = threading.Lock()
        self.create_subscription(
            WrenchStamped,
            '/ft45_joint/wrench',
            self._wrench_cb,
            10,
        )

        self._action_client = ActionClient(self, MoveGroup, '/lbr/move_action')

        self._apply_scene_cli = self.create_client(
            ApplyPlanningScene, '/lbr/apply_planning_scene'
        )

        self.get_logger().info("Esperando action server y servicios...")
        self._action_client.wait_for_server()
        self._apply_scene_cli.wait_for_service()
        self.get_logger().info("Todo listo. Configurando escena...")

        self._setup_planning_scene()

        self.get_logger().info("Iniciando secuencia de presión...")
        self.run_press_sequence()

    def _wrench_cb(self, msg: WrenchStamped):
        with self._wrench_lock:
            self._latest_wrench = msg

    def get_force(self):
        """Devuelve (Fx, Fy, Fz) en Newton o (0,0,0) si no hay datos."""
        with self._wrench_lock:
            if self._latest_wrench is None:
                return 0.0, 0.0, 0.0
            f = self._latest_wrench.wrench.force
            return f.x, f.y, f.z

    def _setup_planning_scene(self):
        from moveit_msgs.msg import PlanningScene, CollisionObject
        from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry
        from std_msgs.msg import Header

        scene = PlanningScene()
        scene.is_diff = True

        surface = CollisionObject()
        surface.header.frame_id = self.BASE_FRAME
        surface.id = "press_surface"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.50, 0.50, 0.10]   # largo, ancho, alto

        pose = Pose()
        pose.position.x = 0.40
        pose.position.y = 0.00
        # centro de la caja queda 5cm por debajo de la cara superior
        pose.position.z = self.SURFACE_Z - 0.05
        pose.orientation.w = 1.0

        surface.primitives      = [box]
        surface.primitive_poses = [pose]
        surface.operation       = CollisionObject.ADD

        scene.world.collision_objects = [surface]

        acm = AllowedCollisionMatrix()

        # todos los links existentes que queremos en la ACM
        entry_names = [
            "press_surface",
            "tool_foam_pad_link",       # la esponja
            "tool_backing_pad_link",    # el backing pad
        ]
        acm.entry_names = entry_names

        # matriz de colision permitida (True = ignorar colision entre ese par)
        for _ in entry_names:
            entry = AllowedCollisionEntry()
            entry.enabled = [True] * len(entry_names)
            acm.entry_values.append(entry)

        scene.allowed_collision_matrix = acm

        req = ApplyPlanningScene.Request()
        req.scene = scene

        future = self._apply_scene_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().success:
            self.get_logger().info("  ✓ Planning scene configurada (superficie + ACM)")
        else:
            self.get_logger().warning("  ⚠ Planning scene no confirmada — continúa igualmente")

    # construccion de goal MoveIt
    def make_pose_goal(
        self,
        x: float, y: float, z: float,
        qx: float = 0.0, qy: float = 1.0, qz: float = 0.0, qw: float = 0.0,
        pos_tol: float = 0.02,
        ang_tol: float = 0.3,
    ) -> MoveGroup.Goal:
        """
        Crea un goal de posición/orientación para EEF_LINK = tool_tcp_link.
        Orientación por defecto: punta mirando hacia abajo (qy=1, qw=0 → 180° en Y).
        """
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.BASE_FRAME
        target_pose.header.stamp    = self.get_clock().now().to_msg()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.x = qx
        target_pose.pose.orientation.y = qy
        target_pose.pose.orientation.z = qz
        target_pose.pose.orientation.w = qw

        # restriccion de posicionamiento
        sphere = SolidPrimitive()
        sphere.type       = SolidPrimitive.SPHERE
        sphere.dimensions = [pos_tol]

        bv = BoundingVolume()
        bv.primitives      = [sphere]
        bv.primitive_poses = [target_pose.pose]

        pos_c                   = PositionConstraint()
        pos_c.header.frame_id   = self.BASE_FRAME
        pos_c.link_name         = self.EEF_LINK
        pos_c.constraint_region = bv
        pos_c.weight            = 1.0

        # restriccion de orientaciones
        ori_c                          = OrientationConstraint()
        ori_c.header.frame_id          = self.BASE_FRAME
        ori_c.link_name                = self.EEF_LINK
        ori_c.orientation              = target_pose.pose.orientation
        ori_c.absolute_x_axis_tolerance = ang_tol
        ori_c.absolute_y_axis_tolerance = ang_tol
        ori_c.absolute_z_axis_tolerance = ang_tol
        ori_c.weight                   = 1.0

        constraints               = Constraints()
        constraints.position_constraints    = [pos_c]
        constraints.orientation_constraints = [ori_c]

        request                          = MotionPlanRequest()
        request.group_name               = self.PLANNING_GROUP
        request.goal_constraints         = [constraints]
        request.num_planning_attempts    = 10
        request.allowed_planning_time    = 10.0
        request.max_velocity_scaling_factor     = 0.2
        request.max_acceleration_scaling_factor = 0.2
        request.pipeline_id              = "ompl"
        request.workspace_parameters.header.frame_id = self.BASE_FRAME
        request.workspace_parameters.min_corner.x = -2.0
        request.workspace_parameters.min_corner.y = -2.0
        request.workspace_parameters.min_corner.z = -0.5
        request.workspace_parameters.max_corner.x = 2.0
        request.workspace_parameters.max_corner.y = 2.0
        request.workspace_parameters.max_corner.z = 2.0

        goal = MoveGroup.Goal()
        goal.request                                        = request
        goal.planning_options.plan_only                     = False
        goal.planning_options.replan                        = True
        goal.planning_options.replan_attempts               = 5
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        return goal

    # envia el goal y espera el resultado
    def send_goal_and_wait(self, goal: MoveGroup.Goal, label: str) -> bool:
        self.get_logger().info(f"→ Enviando: {label}")

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server no disponible")
            return False

        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"  ✗ Goal rechazado: {label}")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        response = result_future.result()
        if response is None:
            self.get_logger().error(f"  ✗ Respuesta None: {label}")
            return False

        ec = response.result.error_code.val
        codes = {
            1: "SUCCESS", -2: "PLANNING_FAILED", -5: "CONTROL_FAILED",
            -7: "TIMED_OUT", -9: "START_STATE_IN_COLLISION",
            -12: "GOAL_IN_COLLISION",
        }
        code_str = codes.get(ec, f"CODE({ec})")

        if ec == 1:
            self.get_logger().info(f"  ✓ {label}: {code_str}")
            return True
        else:
            self.get_logger().error(f"  ✗ {label}: {code_str}")
            return False

    # la secuencia principal del programa
    def run_press_sequence(self):

        # 1. Pre-contacto — TCP justo encima de la superficie
        goal_pre = self.make_pose_goal(x=0.40, y=0.0, z=self.PRECONTACT_Z)
        if not self.send_goal_and_wait(goal_pre, "Pre-contacto"):
            self.get_logger().error("Abortando: no se pudo alcanzar pre-contacto")
            return

        self.get_logger().info("  Fuerza antes de contacto:")
        self._log_force()

        # 2. Presión — baja hasta PRESS_Z (ligero overlap con la caja)
        goal_press = self.make_pose_goal(
            x=0.40, y=0.0, z=self.PRESS_Z,
            pos_tol=0.005,   # tolerancia más fina para la fase de contacto
            ang_tol=0.4,
        )
        # Velocidad reducida para la fase de contacto
        goal_press.request.max_velocity_scaling_factor     = 0.05
        goal_press.request.max_acceleration_scaling_factor = 0.05

        if not self.send_goal_and_wait(goal_press, "Presionando"):
            self.get_logger().error("  ⚠ Fase de presión fallida (puede ser contacto real)")
            # No abortamos — el robot puede haber parado por contacto, igual leemos fuerza

        # 3. Lectura de fuerza durante 2 segundos
        self.get_logger().info("  === Leyendo fuerza durante 2 segundos ===")
        t0 = time.time()
        while time.time() - t0 < 2.0:
            fx, fy, fz = self.get_force()
            self.get_logger().info(
                f"  FT45  Fx={fx:+.2f}N  Fy={fy:+.2f}N  Fz={fz:+.2f}N  "
                f"|F|={( fx**2 + fy**2 + fz**2)**0.5:.2f}N"
            )
            # Parada de seguridad si supera el umbral
            if abs(fz) > self.FORCE_LIMIT_N:
                self.get_logger().warning(
                    f"  ⚠ Umbral de fuerza superado ({abs(fz):.1f} N > {self.FORCE_LIMIT_N} N). Retirando."
                )
                break
            time.sleep(0.2)

        # 4. Retorno
        goal_ret = self.make_pose_goal(x=0.40, y=0.0, z=self.PRECONTACT_Z)
        self.send_goal_and_wait(goal_ret, "Retorno")

        self.get_logger().info("=== Secuencia completada ===")

    def _log_force(self):
        fx, fy, fz = self.get_force()
        self.get_logger().info(f"  FT45  Fx={fx:+.2f}N  Fy={fy:+.2f}N  Fz={fz:+.2f}N")


def main(args=None):
    rclpy.init(args=args)
    node = PressMotionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()