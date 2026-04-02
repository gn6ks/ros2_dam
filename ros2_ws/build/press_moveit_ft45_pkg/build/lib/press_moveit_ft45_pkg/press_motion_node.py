import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    WorkspaceParameters,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    RobotState,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
    Vector3,
)
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration


class PressMotionNode(Node):

    PLANNING_GROUP = "arm"
    EEF_LINK = "lbr_link_ee"
    BASE_FRAME = "lbr_link_0"

    def __init__(self):
        super().__init__('press_motion_node')

        # Action client apuntando al namespace /lbr/move_action
        self._action_client = ActionClient(
            self,
            MoveGroup,
            '/lbr/move_action',
        )

        self.get_logger().info("Esperando a move_group...")
        self._action_client.wait_for_server()
        self.get_logger().info("move_group listo. Iniciando secuencia...")

        self.run_press_sequence()

    def make_pose_goal(self, x: float, y: float, z: float, qx: float = 0.0, qy: float = 0.0, qz: float = 0.0, qw: float = 1.0) -> MoveGroup.Goal:
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

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.BASE_FRAME
        position_constraint.link_name = self.EEF_LINK
        position_constraint.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.05]  # tolerancia 5cm

        bounding_volume = BoundingVolume()
        bounding_volume.primitives = [sphere]
        bounding_volume.primitive_poses = [target_pose.pose]

        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0

        # Orientación con tolerancia amplia
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.BASE_FRAME
        orientation_constraint.link_name = self.EEF_LINK
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.5
        orientation_constraint.absolute_y_axis_tolerance = 0.5
        orientation_constraint.absolute_z_axis_tolerance = 0.5
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [position_constraint]
        constraints.orientation_constraints = [orientation_constraint]

        request = MotionPlanRequest()
        request.group_name = self.PLANNING_GROUP
        request.goal_constraints = [constraints]
        request.num_planning_attempts = 10
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = 0.3
        request.max_acceleration_scaling_factor = 0.3
        request.pipeline_id = "ompl"
        request.workspace_parameters.header.frame_id = self.BASE_FRAME
        request.workspace_parameters.min_corner.x = -2.0
        request.workspace_parameters.min_corner.y = -2.0
        request.workspace_parameters.min_corner.z = -2.0
        request.workspace_parameters.max_corner.x = 2.0
        request.workspace_parameters.max_corner.y = 2.0
        request.workspace_parameters.max_corner.z = 2.0

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        return goal

    def send_goal_and_wait(self, goal: MoveGroup.Goal, label: str) -> bool:
        self.get_logger().info(f"→ Enviando goal: {label}")

        # Comprueba que el servidor está disponible
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server no disponible")
            return False

        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if goal_handle is None:
            self.get_logger().error(f"  ✗ No se obtuvo goal_handle: {label}")
            return False

        if not goal_handle.accepted:
            self.get_logger().error(f"  ✗ Goal RECHAZADO: {label}")
            return False

        self.get_logger().info(f"  ✓ Goal ACEPTADO, esperando resultado...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        response = result_future.result()
        if response is None:
            self.get_logger().error(f"  ✗ Respuesta None: {label}")
            return False

        result = response.result
        error_code = result.error_code.val

        # Tabla de error codes de MoveIt
        error_codes = {
            1:   "SUCCESS",
            -1:  "FAILURE",
            -2:  "PLANNING_FAILED",
            -3:  "INVALID_MOTION_PLAN",
            -4:  "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
            -5:  "CONTROL_FAILED",
            -6:  "UNABLE_TO_AQUIRE_SENSOR_DATA",
            -7:  "TIMED_OUT",
            -8:  "PREEMPTED",
            -9:  "START_STATE_IN_COLLISION",
            -10: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
            -12: "GOAL_IN_COLLISION",
            -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
            -14: "GOAL_CONSTRAINTS_VIOLATED",
            -15: "INVALID_GROUP_NAME",
            -16: "INVALID_GOAL_CONSTRAINTS",
            -17: "INVALID_ROBOT_STATE",
            -18: "INVALID_LINK_NAME",
            -19: "INVALID_OBJECT_ID",
        }

        code_str = error_codes.get(error_code, f"DESCONOCIDO({error_code})")
        
        if error_code == 1:
            self.get_logger().info(f"  ✓ {label}: {code_str}")
            return True
        else:
            self.get_logger().error(f"  ✗ {label}: {code_str} (code={error_code})")
            return False

    def run_press_sequence(self):

        # Posición actual del EEF: x=0.092, y=0.0, z=1.585
        # Orientación actual: x=0.0, y=-0.537, z=0.0, w=0.843
        # Pre-contacto: mantenemos orientación actual, movemos en x hacia adelante

        # 1. Pre-contacto — posición alcanzable cerca de la actual
        goal_pre = self.make_pose_goal(
            x=0.4, y=0.0, z=0.8,
            qx=0.0, qy=-0.537, qz=0.0, qw=0.843   # orientación actual del robot
        )
        ok = self.send_goal_and_wait(goal_pre, "Pre-contacto")
        if not ok:
            self.get_logger().error("Abortando secuencia.")
            return

        # 2. Presión — baja en Z
        goal_press = self.make_pose_goal(
            x=0.4, y=0.0, z=0.5,
            qx=0.0, qy=-0.537, qz=0.0, qw=0.843
        )
        ok = self.send_goal_and_wait(goal_press, "Presionando objeto")
        if not ok:
            self.get_logger().error("Abortando secuencia.")
            return

        # 3. Retorno
        goal_ret = self.make_pose_goal(
            x=0.4, y=0.0, z=0.8,
            qx=0.0, qy=-0.537, qz=0.0, qw=0.843
        )
        self.send_goal_and_wait(goal_ret, "Retorno")

        self.get_logger().info("=== Secuencia completada ===")


def main(args=None):
    rclpy.init(args=args)
    node = PressMotionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()