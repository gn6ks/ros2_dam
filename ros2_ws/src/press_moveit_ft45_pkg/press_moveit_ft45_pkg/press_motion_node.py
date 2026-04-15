import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    AllowedCollisionEntry,
)
from moveit_msgs.srv import ApplyPlanningScene
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import WrenchStamped
import threading
import time


class PressMotionNode(Node):

    PLANNING_GROUP = "arm"
    EEF_LINK       = "tool_tcp_link"
    BASE_FRAME     = "lbr_link_0"

    SURFACE_Z      = 0.45
    PRESS_Z        = 0.44
    PRECONTACT_Z   = 0.55
    FORCE_LIMIT_N  = 30.0

    def __init__(self):
        super().__init__('press_motion_node')

        self._latest_wrench = None
        self._wrench_lock   = threading.Lock()

        # topico correcto segun el bridge configurado en gazebo
        self.create_subscription(
            WrenchStamped,
            '/ft45/ft45_ft_sensor',
            self._wrench_cb,
            10,
        )

        self._action_client   = ActionClient(self, MoveGroup, '/lbr/move_action')
        self._apply_scene_cli = self.create_client(ApplyPlanningScene, '/lbr/apply_planning_scene')

        self.get_logger().info("Esperando a move_group...")
        self._action_client.wait_for_server()
        self._apply_scene_cli.wait_for_service()
        self.get_logger().info("move_group listo. Iniciando secuencia...")

        self._setup_planning_scene()
        self.run_press_sequence()

    def _wrench_cb(self, msg: WrenchStamped):
        with self._wrench_lock:
            self._latest_wrench = msg

    def get_force(self):
        with self._wrench_lock:
            if self._latest_wrench is None:
                return 0.0, 0.0, 0.0
            f = self._latest_wrench.wrench.force
            return f.x, f.y, f.z

    def _setup_planning_scene(self):
        from moveit_msgs.msg import PlanningScene, CollisionObject

        scene = PlanningScene()
        scene.is_diff = True

        # caja sobre la que presiona el robot
        surface = CollisionObject()
        surface.header.frame_id = self.BASE_FRAME
        surface.id = "press_surface"

        box = SolidPrimitive()
        box.type       = SolidPrimitive.BOX
        box.dimensions = [0.50, 0.50, 0.10]

        pose = Pose()
        pose.position.x    = 0.40
        pose.position.y    = 0.00
        pose.position.z    = self.SURFACE_Z - 0.05
        pose.orientation.w = 1.0

        surface.primitives      = [box]
        surface.primitive_poses = [pose]
        surface.operation       = CollisionObject.ADD

        scene.world.collision_objects = [surface]

        # ignorar colisiones entre la superficie y los links del tool
        from moveit_msgs.msg import AllowedCollisionMatrix
        acm = AllowedCollisionMatrix()
        entry_names = ["press_surface", "tool_foam_pad_link", "tool_backing_pad_link"]
        acm.entry_names = entry_names
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
            self.get_logger().info("planning scene configurada")
        else:
            self.get_logger().warning("planning scene no confirmada, continuando")

    def make_pose_goal(
        self,
        x: float, y: float, z: float,
        qx: float = 0.0, qy: float = 1.0, qz: float = 0.0, qw: float = 0.0,
        pos_tol: float = 0.02,
        ang_tol: float = 0.3,
    ) -> MoveGroup.Goal:
        # orientacion por defecto: tcp apuntando hacia abajo
        target_pose = PoseStamped()
        target_pose.header.frame_id    = self.BASE_FRAME
        target_pose.header.stamp       = self.get_clock().now().to_msg()
        target_pose.pose.position.x    = x
        target_pose.pose.position.y    = y
        target_pose.pose.position.z    = z
        target_pose.pose.orientation.x = qx
        target_pose.pose.orientation.y = qy
        target_pose.pose.orientation.z = qz
        target_pose.pose.orientation.w = qw

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

        ori_c                            = OrientationConstraint()
        ori_c.header.frame_id            = self.BASE_FRAME
        ori_c.link_name                  = self.EEF_LINK
        ori_c.orientation                = target_pose.pose.orientation
        ori_c.absolute_x_axis_tolerance  = ang_tol
        ori_c.absolute_y_axis_tolerance  = ang_tol
        ori_c.absolute_z_axis_tolerance  = ang_tol
        ori_c.weight                     = 1.0

        constraints = Constraints()
        constraints.position_constraints    = [pos_c]
        constraints.orientation_constraints = [ori_c]

        request = MotionPlanRequest()
        request.group_name                      = self.PLANNING_GROUP
        request.goal_constraints                = [constraints]
        request.num_planning_attempts           = 10
        request.allowed_planning_time           = 10.0
        request.max_velocity_scaling_factor     = 0.3
        request.max_acceleration_scaling_factor = 0.3
        request.pipeline_id                     = "ompl"
        request.workspace_parameters.header.frame_id = self.BASE_FRAME
        request.workspace_parameters.min_corner.x = -2.0
        request.workspace_parameters.min_corner.y = -2.0
        request.workspace_parameters.min_corner.z = -0.5
        request.workspace_parameters.max_corner.x =  2.0
        request.workspace_parameters.max_corner.y =  2.0
        request.workspace_parameters.max_corner.z =  2.0

        goal = MoveGroup.Goal()
        goal.request                                              = request
        goal.planning_options.plan_only                           = False
        goal.planning_options.replan                              = True
        goal.planning_options.replan_attempts                     = 5
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        return goal

    def send_goal_and_wait(self, goal: MoveGroup.Goal, label: str) -> bool:
        self.get_logger().info(f"-> Enviando goal: {label}")

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("action server no disponible")
            return False

        future      = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"goal rechazado: {label}")
            return False

        self.get_logger().info(f"  goal aceptado, esperando resultado...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        response = result_future.result()

        if response is None:
            self.get_logger().error(f"respuesta None: {label}")
            return False

        ec = response.result.error_code.val
        codes = {
            1: "SUCCESS", -2: "PLANNING_FAILED", -5: "CONTROL_FAILED",
            -7: "TIMED_OUT", -9: "START_STATE_IN_COLLISION", -12: "GOAL_IN_COLLISION",
        }
        code_str = codes.get(ec, f"CODE({ec})")

        if ec == 1:
            self.get_logger().info(f"  {label}: {code_str}")
            return True
        else:
            self.get_logger().error(f"  {label}: {code_str}")
            return False

    def _log_force(self):
        fx, fy, fz = self.get_force()
        self.get_logger().info(f"  FT45  Fx={fx:+.2f}N  Fy={fy:+.2f}N  Fz={fz:+.2f}N")

    def run_press_sequence(self):

        # fase 1: posicion segura sobre la superficie
        goal_pre = self.make_pose_goal(x=0.40, y=0.0, z=self.PRECONTACT_Z)
        if not self.send_goal_and_wait(goal_pre, "Pre-contacto"):
            self.get_logger().error("abortando: no se pudo alcanzar pre-contacto")
            return

        self._log_force()

        # fase 2: bajada hasta contacto con velocidad reducida
        goal_press = self.make_pose_goal(
            x=0.40, y=0.0, z=self.PRESS_Z,
            pos_tol=0.005,
            ang_tol=0.4,
        )
        goal_press.request.max_velocity_scaling_factor     = 0.05
        goal_press.request.max_acceleration_scaling_factor = 0.05

        if not self.send_goal_and_wait(goal_press, "Presionando objeto"):
            self.get_logger().warning("fase de presion fallida (posible contacto real)")

        # fase 3: lectura de fuerza durante 2 segundos
        self.get_logger().info("leyendo fuerza durante 2 segundos...")
        t0 = time.time()
        while time.time() - t0 < 2.0:
            fx, fy, fz = self.get_force()
            self.get_logger().info(
                f"  FT45  Fx={fx:+.2f}N  Fy={fy:+.2f}N  Fz={fz:+.2f}N  "
                f"|F|={(fx**2 + fy**2 + fz**2)**0.5:.2f}N"
            )
            if abs(fz) > self.FORCE_LIMIT_N:
                self.get_logger().warning(f"umbral superado ({abs(fz):.1f}N > {self.FORCE_LIMIT_N}N), retirando")
                break
            time.sleep(0.2)

        # fase 4: retorno a posicion segura
        goal_ret = self.make_pose_goal(x=0.40, y=0.0, z=self.PRECONTACT_Z)
        self.send_goal_and_wait(goal_ret, "Retorno")

        self.get_logger().info("secuencia completada")


def main(args=None):
    rclpy.init(args=args)
    node = PressMotionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()