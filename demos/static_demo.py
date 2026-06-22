#!/usr/bin/env python3

import copy
import math
import sys
import threading
import time
import xml.etree.ElementTree as ET
from collections import deque

import PyKDL
import rclpy
import rclpy.duration
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, WrenchStamped
from moveit_msgs.msg import DisplayTrajectory, RobotState, RobotTrajectory

# pymoveit2: stable wrapper around the MoveIt2 API
from pymoveit2 import MoveIt2

# from pymoveit2.robots import (
#     iiwa7 as robot_config,  # change to your robot module if different
# )
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Header
from tf_transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectoryPoint


# FT sensor: lista de topics candidatos a probar, en orden. Ajusta según
# lo que confirmes con `ros2 topic list` en tu /lbr namespace (puede que
# sea "/lbr/ft_sensor/wrench" en vez de "/ft_sensor/wrench" sin namespace).
FT_TOPIC_CANDIDATES = [
    "/ft_sensor/wrench",
    "/lbr/ft_sensor/wrench",
]

WINDOW_SIZE = 5


def all_close(goal, actual, tolerance):
    if isinstance(goal, list):
        for i in range(len(goal)):
            if abs(actual[i] - goal[i]) > tolerance:
                return False
        return True
    elif isinstance(goal, PoseStamped):
        return all_close(goal.pose, actual.pose, tolerance)
    elif isinstance(goal, Pose):

        def _to_list(p):
            return [
                p.position.x,
                p.position.y,
                p.position.z,
                p.orientation.x,
                p.orientation.y,
                p.orientation.z,
                p.orientation.w,
            ]

        return all_close(_to_list(goal), _to_list(actual), tolerance)
    return True


def ask_speed(default: float = 50.0, max_recommended: float = 300.0) -> float:
    """
    Ask the user for the Cartesian EEF speed (mm/s) to use for the demo.

    Validates the input is numeric and positive.  If the value exceeds the
    recommended maximum, warns that MoveIt2 may reject the plan (joint limits
    exceeded, Jacobian near singularity) and requests
    confirmation before continuing.
    """
    while True:
        raw = input(
            f"Cartesian EEF speed in mm/s [Enter = {default}]: "
        ).strip()

        if raw == "":
            return default

        try:
            speed = float(raw)
        except ValueError:
            print("  -> Non-numeric value. Valid example: 50 or 70.5")
            continue

        if speed <= 0:
            print("  -> Speed must be a positive number.")
            continue

        if speed > max_recommended:
            confirm = (
                input(
                    f"  -> {speed} mm/s is high; MoveIt2 may reject the plan for "
                    "exceed the URDF joint limits. Continue? [y/N]: "
                )
                .strip()
                .lower()
            )
            if confirm != "s":
                continue

        return speed


class EEF:
    def __init__(
        self,
        EE_end_frame=PyKDL.Frame(),
        x=0,
        y=0,
        z=0,
        ATC_frame=PyKDL.Frame(),
        name="",
        path="",
    ):
        self.EE_end_frame = EE_end_frame
        self.x = x
        self.y = y
        self.z = z
        self.ATC_frame = ATC_frame
        self.name = name
        self.path = path


class MoveGroupPythonIntefaceControl(Node):
    JOINT_NAMES = [
        "lbr_A1",
        "lbr_A2",
        "lbr_A3",
        "lbr_A4",
        "lbr_A5",
        "lbr_A6",
        "lbr_A7",
    ]
    BASE_LINK = "lbr_link_0"
    EEF_LINK = "lbr_link_ee"
    GROUP_NAME = "arm"

    def __init__(self):
        super().__init__("move_group_control", namespace="/lbr")

        # call pymoveit2
        self._moveit2 = MoveIt2(
            node=self,
            joint_names=self.JOINT_NAMES,
            base_link_name=self.BASE_LINK,
            end_effector_name=self.EEF_LINK,
            group_name=self.GROUP_NAME,
        )

        from moveit_msgs.srv import GetCartesianPath

        self._cartesian_client = self.create_client(
            GetCartesianPath,
            "/lbr/compute_cartesian_path",  # adjust namespace if changed
        )
        if not self._cartesian_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                "Service /compute_cartesian_path not available yet."
            )

        # Trajectory publisher for RViz2
        self._display_traj_pub = self.create_publisher(
            DisplayTrajectory, "display_planned_path", 20
        )

        # End-effector link name (for FK)
        self.eef_link = self.EEF_LINK

        # Cached joint velocity limits (parsed from URDF once)
        self._vel_limit: dict | None = None

        # Planning scene object (to read URDF and compute FK)
        # pymoveit2 expone el robot_model a traves de MoveIt2.robot_model
        self.box_name = ""

        self._wrench_lock = threading.Lock()
        self._active_ft_topic: str | None = None
        self._ft_sub = None
        self._fx_window: deque = deque(maxlen=WINDOW_SIZE)
        self._fy_window: deque = deque(maxlen=WINDOW_SIZE)
        self._fz_window: deque = deque(maxlen=WINDOW_SIZE)
        self._discover_ft_topic()
        if self._active_ft_topic is None:
            self.get_logger().warn(
                "no FT topic in activity "
            )

        self.get_logger().info("MoveGroupPythonIntefaceControl ready.")

    def _discover_ft_topic(self, timeout_per_topic: float = 2.0):
        self.get_logger().info("searching FT active topic")

        for topic in FT_TOPIC_CANDIDATES:
            self.get_logger().info(f"  trying: {topic}")
            found = threading.Event()

            def _cb(msg: WrenchStamped, t=topic, ev=found):
                with self._wrench_lock:
                    self._active_ft_topic = t
                ev.set()

            sub = self.create_subscription(WrenchStamped, topic, _cb, 10)

            deadline = time.time() + timeout_per_topic
            while time.time() < deadline and not found.is_set():
                rclpy.spin_once(self, timeout_sec=0.1)

            if found.is_set():
                self.get_logger().info(f"  -> topic FT active: {topic}")
                self.destroy_subscription(sub)
                self._ft_sub = self.create_subscription(
                    WrenchStamped, topic, self._wrench_cb, 10
                )
                return
            self.destroy_subscription(sub)

        self.get_logger().warn("no FT with data published")

    def _wrench_cb(self, msg: WrenchStamped):
        with self._wrench_lock:
            f = msg.wrench.force
            self._fx_window.append(f.x)
            self._fy_window.append(f.y)
            self._fz_window.append(f.z)

    def get_force(self) -> tuple[float, float, float]:
        with self._wrench_lock:
            if not self._fx_window:
                return 0.0, 0.0, 0.0
            fx = sum(self._fx_window) / len(self._fx_window)
            fy = sum(self._fy_window) / len(self._fy_window)
            fz = sum(self._fz_window) / len(self._fz_window)
        return fx, fy, fz

    def reset_force(self, settle_time: float = 0.3) -> bool:
        if self._active_ft_topic is None:
            self.get_logger().warn(
                "reset_force: no hay topic FT activo, no se puede resetear."
            )
            return False

        with self._wrench_lock:
            self._fx_window.clear()
            self._fy_window.clear()
            self._fz_window.clear()

        deadline = time.time() + max(settle_time, 0.05)
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        with self._wrench_lock:
            filled = len(self._fx_window) >= self._fx_window.maxlen

        if not filled:
            self.get_logger().warn(
                "reset_force: window not entitled "
                f"{settle_time:.2f}s — no Mhz correct"
            )
        return filled

    def wait_for_joint_state(self, timeout_sec: float = 10.0) -> bool:
        """
        Wait for at least one /joint_states message to arrive.

        Right after node creation, rclpy has not yet run any spin, so the
        internal pymoveit2 subscription to /joint_states has not triggered
        any callback and self._moveit2.joint_state is still None. If
        planning is attempted before this, go_to_pose()/go_to_pose_speed()
        fail immediately with "No joint_state available", regardless of the
        requested speed. This method calls spin_once() until the first
        message arrives or the timeout expires.
        """
        start = self.get_clock().now()
        while self._moveit2.joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed_sec = (self.get_clock().now() - start).nanoseconds * 1e-9
            if elapsed_sec > timeout_sec:
                return False
        return True

    def go_to_joint_state(self):
        """Moves the robot to the fixed joint configuration used in the benchmark."""
        joint_goal = [
            90.0 * math.pi / 180.0,
            0.0 * math.pi / 180.0,
            0.0 * math.pi / 180.0,
            -90.0 * math.pi / 180.0,
            0.0 * math.pi / 180.0,
            90.0 * math.pi / 180.0,
            0.0,
        ]
        self._moveit2.move_to_configuration(joint_goal)
        self._moveit2.wait_until_executed()
        return True

    def go_to_pose(self, pose: Pose) -> bool:
        """
        Plan and execute a move (without speed control) to a Cartesian pose
        using pymoveit2. Catches any MoveIt2/ROS2 exception and reports it
        without propagating, returning False.
        """
        try:
            self._moveit2.move_to_pose(
                position=[pose.position.x, pose.position.y, pose.position.z],
                quat_xyzw=[
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ],
            )
            self._moveit2.wait_until_executed()
        except Exception as exc:
            self.get_logger().error(f"Exception in go_to_pose: {exc!r}")
            return False
        return True

    def follow_trajectory_speed(
        self, waypoints, speeds, ang_speeds=[], accel=100.0
    ) -> bool:
        """
        Compute and execute a multi-section trajectory with speed control.
        Never raises an exception: any failure is reported via the logger and
        False is returned so the caller can decide how to proceed.
        """
        try:
            plan, success = self.compute_cartesian_path_velocity_control(
                waypoints, speeds, EE_ang_speed=ang_speeds, max_linear_accel=accel
            )
        except Exception as exc:
            self.get_logger().error(
                f"Exception computing multi-section trajectory: {exc!r}"
            )
            return False

        if plan is None:
            self.get_logger().error(
                "Could not plan the multi-section trajectory "
                "(check reachability, collisions, or joint limits)."
            )
            return False

        if not success:
            self.get_logger().warn(
                "The speed profile was not fully satisfied; "
                "executing the available plan anyway."
            )

        executed = self._publish_and_execute(plan, success)
        time.sleep(0.5)
        return executed

    def go_to_pose_speed(
        self, pose: Pose, speed=10.0, ang_speed=[], accel=100.0
    ) -> bool:
        """
        Plan and execute a Cartesian move at a given speed (mm/s).

        Returns True if the plan was computed and executed successfully, False
        otherwise. Does not propagate exceptions: any MoveIt2 error (service
        unavailable, null plan, rejected goal, controller failure...) is caught,
        logged and returned as False, so the demo can proceed with the next
        step instead of crashing.
        """
        try:
            current_pose = self._get_current_eef_pose()
            waypoints = [[copy.deepcopy(current_pose), copy.deepcopy(pose)]]
            plan, success = self.compute_cartesian_path_velocity_control(
                waypoints, [speed], EE_ang_speed=ang_speed, max_linear_accel=accel
            )
        except Exception as exc:
            self.get_logger().error(
                f"Exception computing trajectory at {speed} mm/s: {exc!r}"
            )
            return False

        if plan is None:
            self.get_logger().error(
                f"Could not plan move at {speed} mm/s "
                "(check joint limits, collisions, or reachability)."
            )
            return False

        if not success:
            self.get_logger().warn(
                f"El plan a {speed} mm/s presenta advertencias "
                "(e.g. target speed not reached due to acceleration or "
                "joint limits). Executing anyway."
            )

        executed = self._publish_and_execute(plan, success)
        time.sleep(0.5)
        return executed

    def _publish_and_execute(self, plan, success: bool) -> bool:
        """
        Publish the plan to RViz2 and execute it via the FollowJointTrajectory
        action. Returns True only if the controller confirms a successful
        execution; in any other case (service unavailable, goal rejected,
        timeout, controller error, or exception) returns False and logs the
        reason, without propagating the exception to the caller.
        """
        if plan is None:
            self.get_logger().error("Could not compute or publish the plan.")
            return False

        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory.append(plan)
        self._display_traj_pub.publish(display_trajectory)
        self.get_logger().info("Plan published to RViz2.")

        if not success:
            self.get_logger().warn(
                "The speed calculation was not fully successful "
                "(ver advertencias previas); se intenta ejecutar igualmente."
            )

        try:
            from action_msgs.msg import GoalStatus
            from control_msgs.action import FollowJointTrajectory
            from rclpy.action import ActionClient

            if not hasattr(self, "_fjt_client"):
                self._fjt_client = ActionClient(
                    self,
                    FollowJointTrajectory,
                    "/lbr/joint_trajectory_controller/follow_joint_trajectory",
                )

            if not self._fjt_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error(
                    "Action server follow_joint_trajectory not available "
                    "(is controller_manager running? is namespace correct?)."
                )
                return False

            goal = FollowJointTrajectory.Goal()
            # plan.joint_trajectory.header.stamp = self.get_clock().now().to_msg()
            
            from builtin_interfaces.msg import Time
            plan.joint_trajectory.header.stamp = Time(sec=0, nanosec=0) 
            
            plan.joint_trajectory.header.frame_id = self.BASE_LINK
            goal.trajectory = plan.joint_trajectory

            # Validate monotonicity and values before sending to controller
            valid, error_msg = self._validate_trajectory(plan)
            if not valid:
                self.get_logger().error(
                    f"Invalid trajectory before sending to controller: "
                    f"{error_msg}. Se descarta este plan."
                )
                return False

            self.get_logger().info(
                f"Plan final | waypoints={len(plan.joint_trajectory.points)} | "
                f"duration={plan.joint_trajectory.points[-1].time_from_start.sec + plan.joint_trajectory.points[-1].time_from_start.nanosec * 1e-9:.2f}s"
            )

            future = self._fjt_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            goal_handle = future.result()

            if goal_handle is None:
                self.get_logger().error(
                    "No response from action server when sending goal "
                    "(timeout de 30s)."
                )
                return False

            if not goal_handle.accepted:
                self.get_logger().error(
                    "Goal rejected by the controller "
                    "(trajectory out of limits, non-monotonic times, etc.)."
                )
                return False

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
            wrapped_result = result_future.result()

            if wrapped_result is None:
                self.get_logger().error(
                    "No result from controller after execution "
                    "(timeout de 60s)."
                )
                return False

            if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().error(
                    f"Execution did not complete successfully (status={wrapped_result.status})."
                )
                return False

            error_code = wrapped_result.result.error_code
            if error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                self.get_logger().error(
                    f"Controller returned error code ({error_code}): "
                    f"{wrapped_result.result.error_string}"
                )
                return False

            self.get_logger().info("Trajectory executed successfully.")
            return True

        except Exception as exc:
            self.get_logger().error(
                f"Exception during trajectory execution: {exc!r}"
            )
            return False

    def _validate_trajectory(self, trajectory) -> tuple:
        """
        Validate that a RobotTrajectory can be executed by FollowJointTrajectory.
        Returns (ok: bool, error_message: str).

        Checks:
            - At least 2 waypoints
            - First waypoint has time_from_start = 0
            - time_from_start is strictly increasing
            - No NaN or inf in positions, velocities, or accelerations
        """
        pts = trajectory.joint_trajectory.points
        if len(pts) < 2:
            return False, "Trajectory has fewer than 2 waypoints"

        prev_time = pts[0].time_from_start
        if prev_time.sec != 0 or prev_time.nanosec != 0:
            return False, (
                f"First waypoint does not have time_from_start=0 "
                f"(sec={prev_time.sec}, nanosec={prev_time.nanosec})"
            )

        for i, pt in enumerate(pts[1:], 1):
            t = pt.time_from_start
            t_sec = t.sec + t.nanosec * 1e-9
            prev_sec = prev_time.sec + prev_time.nanosec * 1e-9

            if t_sec <= prev_sec:
                return False, (
                    f"Non-monotonic times at waypoint {i}: "
                    f"{prev_sec:.9f} -> {t_sec:.9f} "
                    f"(diff={t_sec - prev_sec:.9f}s)"
                )

            for j, pos in enumerate(pt.positions):
                if math.isnan(pos) or math.isinf(pos):
                    return False, (
                        f"Position {j} of waypoint {i} is {pos}"
                    )

            for j, vel in enumerate(pt.velocities):
                if math.isnan(vel) or math.isinf(vel):
                    return False, (
                        f"Velocity {j} of waypoint {i} is {vel}"
                    )

            for j, acc in enumerate(pt.accelerations):
                if math.isnan(acc) or math.isinf(acc):
                    return False, (
                        f"Acceleration {j} of waypoint {i} is {acc}"
                    )

            prev_time = t

        return True, ""

    def _get_robot_description(self) -> str:
        """
        Retrieve robot_description from the move_group node.

        In this ROS2 + lbr_fri_ros2_stack setup the parameter lives on
        /lbr/move_group, not on robot_state_publisher nor locally.
        Falls back to robot_state_publisher as a secondary source.
        """
        from rcl_interfaces.srv import GetParameters

        service_candidates = [
            "/lbr/move_group/get_parameters",
            "/lbr/robot_state_publisher/get_parameters",
        ]
        for srv_name in service_candidates:
            try:
                gp_client = self.create_client(GetParameters, srv_name)
                if gp_client.wait_for_service(timeout_sec=2.0):
                    req = GetParameters.Request()
                    req.names = ["robot_description"]
                    future = gp_client.call_async(req)
                    rclpy.spin_until_future_complete(
                        self, future, timeout_sec=2.0
                    )
                    result = future.result()
                    if result and result.values:
                        self.get_logger().info(
                            f"robot_description retrieved via {srv_name}."
                        )
                        return result.values[0].string_value
            except Exception:
                continue

        self.get_logger().warn(
            "robot_description not found. "
            "Joint velocity limits from URDF will not be available "
            "for enforcement."
        )
        return ""

    def _get_current_eef_pose(self) -> Pose:
        """Return the current EEF pose via FK over the current joint_state."""
        js = self._moveit2.joint_state
        if js is None:
            # Short retry: if the topic dropped messages for a moment
            # (e.g. after a long movement), give a second chance before
            # declaring failure.
            self.wait_for_joint_state(timeout_sec=2.0)
            js = self._moveit2.joint_state
        if js is None:
            self.get_logger().error(
                "No joint_state available (no /joint_states messages)."
            )
            return Pose()
        pose = self._fk_from_joint_positions(list(js.name), list(js.position))
        if pose is None:
            self.get_logger().error("FK failed for current state.")
            return Pose()
        return pose

    def _fk_from_joint_positions(
        self, joint_names: list, joint_positions: list
    ) -> Pose | None:
        """
        FK for a given set of joint positions.

        pymoveit2 does not expose FK directly, but we can use the MoveIt2
        /compute_fk service via rclpy.
        This method replaces the iiwa_fk_server call from the original.
        """
        from moveit_msgs.srv import GetPositionFK
        from std_msgs.msg import Header as StdHeader

        if not hasattr(self, "_fk_client"):
            self._fk_client = self.create_client(GetPositionFK, "/lbr/compute_fk")
            if not self._fk_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("Service /compute_fk not available.")
                return None

        req = GetPositionFK.Request()
        req.header.frame_id = self.BASE_LINK
        req.header.stamp = self.get_clock().now().to_msg()
        req.fk_link_names = [self.eef_link]

        rs = RobotState()
        rs.joint_state.name = list(joint_names)
        rs.joint_state.position = list(joint_positions)
        req.robot_state = rs

        future = self._fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        result = future.result()

        if result is None or not result.pose_stamped:
            return None
        return result.pose_stamped[0].pose

    def frame_to_pose(self, frame: PyKDL.Frame) -> Pose:
        pose = Pose()
        pose.position.x = frame.p[0]
        pose.position.y = frame.p[1]
        pose.position.z = frame.p[2]
        ang = frame.M.GetQuaternion()
        pose.orientation.x = ang[0]
        pose.orientation.y = ang[1]
        pose.orientation.z = ang[2]
        pose.orientation.w = ang[3]
        return pose

    @staticmethod
    def _pose_to_mm(pose: Pose) -> Pose:
        """Return a copy of the pose with positions converted to mm."""
        p = copy.deepcopy(pose)
        p.position.x *= 1000
        p.position.y *= 1000
        p.position.z *= 1000
        return p

    def pose_to_frame(self, pose: Pose) -> PyKDL.Frame:
        frame = PyKDL.Frame()
        frame.p = PyKDL.Vector(pose.position.x, pose.position.y, pose.position.z)
        frame.M = PyKDL.Rotation.Quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        return frame

    def get_transpose_rot(self, rot: PyKDL.Rotation) -> PyKDL.Rotation:
        return PyKDL.Rotation(
            rot[0, 0],
            rot[1, 0],
            rot[2, 0],
            rot[0, 1],
            rot[1, 1],
            rot[2, 1],
            rot[0, 2],
            rot[1, 2],
            rot[2, 2],
        )

    def get_inverse_frame(self, frame: PyKDL.Frame) -> PyKDL.Frame:
        inv = PyKDL.Frame()
        M, p = frame.M, frame.p
        x = -(p[0] * M[0, 0] + p[1] * M[1, 0] + p[2] * M[2, 0])
        y = -(p[0] * M[0, 1] + p[1] * M[1, 1] + p[2] * M[2, 1])
        z = -(p[0] * M[0, 2] + p[1] * M[1, 2] + p[2] * M[2, 2])
        inv.p = PyKDL.Vector(x, y, z)
        inv.M = self.get_transpose_rot(M)
        return inv

    def compute_distance(self, pose1: Pose, pose2: Pose) -> float:
        return math.sqrt(
            (pose1.position.x - pose2.position.x) ** 2
            + (pose1.position.y - pose2.position.y) ** 2
            + (pose1.position.z - pose2.position.z) ** 2
        )

    def compute_angle_distance(self, pose1: Pose, pose2: Pose) -> float:
        f1 = self.pose_to_frame(pose1)
        f2 = self.pose_to_frame(pose2)
        return abs((f1.Inverse() * f2).M.GetRotAngle()[0])

    def compute_lin_or_ang_distance(
        self, pose1: Pose, pose2: Pose, linear=True
    ) -> float:
        return (
            self.compute_distance(pose1, pose2)
            if linear
            else self.compute_angle_distance(pose1, pose2)
        )

    def get_shifted_pose(self, origin_pose: Pose, shift: list) -> Pose:
        tf_origin = self.pose_to_frame(origin_pose)
        tf_shift = PyKDL.Frame()
        tf_shift.p = PyKDL.Vector(shift[0], shift[1], shift[2])
        tf_shift.M.DoRotX(shift[3])
        tf_shift.M.DoRotY(shift[4])
        tf_shift.M.DoRotZ(shift[5])
        return self.frame_to_pose(tf_origin * tf_shift)

    def degree_difference(self, R1: PyKDL.Rotation, R2: PyKDL.Rotation):
        R_1_2 = self.get_transpose_rot(R1) * R2
        rad_dif = R_1_2.GetRPY()
        deg_dif = [r * (180.0 / math.pi) for r in rad_dif]
        return deg_dif, rad_dif

    def interpolate_trajectory(
        self, initial_pose, final_pose, step_pos_min, step_deg_min, n_points_max
    ):
        waypoints = []
        if initial_pose == final_pose:
            self.get_logger().warn("Cannot interpolate: same pose.")
            return False, waypoints

        step_pos_min = float(step_pos_min)
        step_deg_min = float(step_deg_min)
        pos_dif = self.compute_distance(initial_pose, final_pose)
        deg_dif, rad_dif = self.degree_difference(
            self.pose_to_frame(initial_pose).M,
            self.pose_to_frame(final_pose).M,
        )
        n_points = n_points_max
        candidates = [
            pos_dif / step_pos_min,
            abs(deg_dif[0]) / step_deg_min,
            abs(deg_dif[1]) / step_deg_min,
            abs(deg_dif[2]) / step_deg_min,
        ]
        if max(candidates) < 20:
            n_points = int(max(candidates))

        waypoints.append(initial_pose)
        for point in range(n_points):
            if point > 0:
                x = initial_pose.position.x + (
                    (final_pose.position.x - initial_pose.position.x)
                    * float(point)
                    / float(n_points)
                )
                y = initial_pose.position.y + (
                    (final_pose.position.y - initial_pose.position.y)
                    * float(point)
                    / float(n_points)
                )
                z = initial_pose.position.z + (
                    (final_pose.position.z - initial_pose.position.z)
                    * float(point)
                    / float(n_points)
                )
                rotation = self.pose_to_frame(initial_pose).M
                rotation.DoRotX(rad_dif[0] * float(point) / float(n_points))
                rotation.DoRotY(rad_dif[1] * float(point) / float(n_points))
                rotation.DoRotZ(rad_dif[2] * float(point) / float(n_points))
                new_frame = PyKDL.Frame()
                new_frame.p = PyKDL.Vector(x, y, z)
                new_frame.M = rotation
                waypoints.append(self.frame_to_pose(new_frame))
        waypoints.append(final_pose)
        return True, waypoints

    def adjust_plan_speed(
        self,
        traj_poses,
        EE_speed_aux,
        v_change,
        traj_mov,
        max_accel,
        all_plans,
        linear=True,
    ):
        thres = 0.05
        if not linear:
            thres *= 0.7 * (math.pi / 180)

        n_joints = len(all_plans[0].joint_trajectory.points[0].positions)
        zero_Jvel = [0.0] * n_joints
        corrected_traj = [
            {
                "pose": traj_poses[0][0],
                "state": all_plans[0].joint_trajectory.points[0].positions,
                "EE_speed": 0,
                "EE_accel": 0,
                "time": 0,
                "Jspeed": copy.deepcopy(zero_Jvel),
                "Jaccel": copy.deepcopy(zero_Jvel),
            }
        ]

        success = True
        i = 0
        s_i = 1
        v_chg_i = 0
        init_speed_change = False
        final_speed_change = False
        speed_diff = 0.0  # initialized at the start of each plan segment
        trans_accel = 0.0  # used in final_speed_change block

        for plan_poses in traj_poses:
            if traj_mov[i] < thres:
                j = -1
                for pose in plan_poses:
                    j += 1
                    if pose == corrected_traj[-1]["pose"]:
                        continue
                    corrected_traj.append(
                        {
                            "pose": pose,
                            "state": all_plans[i].joint_trajectory.points[j].positions,
                            "EE_speed": 0,
                            "EE_accel": 0,
                            "time": corrected_traj[-1]["time"],
                            "Jspeed": copy.deepcopy(zero_Jvel),
                            "Jaccel": copy.deepcopy(zero_Jvel),
                        }
                    )
                i += 1
                s_i += 1
                continue

            init_speed_change = EE_speed_aux[s_i] > EE_speed_aux[s_i - 1]
            final_speed_change = EE_speed_aux[s_i] > EE_speed_aux[s_i + 1]
            j = -1
            x_plan = 0
            init_transition_indexes = []
            first_final = True

            for pose in plan_poses:
                j += 1
                if pose == corrected_traj[-1]["pose"]:
                    continue

                if (
                    self.compute_lin_or_ang_distance(
                        pose, corrected_traj[-1]["pose"], linear
                    )
                    < thres
                    and not init_speed_change
                ):
                    corrected_traj.append(
                        {
                            "pose": corrected_traj[-1]["pose"],
                            "state": all_plans[i].joint_trajectory.points[j].positions,
                            "EE_speed": 0,
                            "EE_accel": 0,
                            "time": corrected_traj[-1]["time"],
                            "Jspeed": copy.deepcopy(zero_Jvel),
                            "Jaccel": copy.deepcopy(zero_Jvel),
                        }
                    )
                    continue

                if init_speed_change:
                    if final_speed_change:
                        if (
                            v_change[v_chg_i]["x_min_req"]
                            + v_change[v_chg_i + 1]["x_min_req"]
                        ) > traj_mov[i]:
                            t2 = (
                                -2 * EE_speed_aux[s_i + 1]
                                + math.sqrt(
                                    2
                                    * (
                                        EE_speed_aux[s_i + 1] ** 2
                                        + EE_speed_aux[s_i - 1] ** 2
                                        + 2 * max_accel * traj_mov[i]
                                    )
                                )
                            ) / (2 * max_accel)
                            v_change[v_chg_i + 1]["x_min_req"] = (
                                EE_speed_aux[s_i + 1] + max_accel * t2
                            ) * t2 - 0.5 * max_accel * t2**2
                            v_change[v_chg_i]["x_min_req"] = (
                                traj_mov[i] - v_change[v_chg_i + 1]["x_min_req"]
                            )

                    x_plan += self.compute_lin_or_ang_distance(
                        pose, corrected_traj[-1]["pose"], linear
                    )
                    corrected_traj.append(
                        {
                            "pose": pose,
                            "state": all_plans[i].joint_trajectory.points[j].positions,
                            "EE_speed": 0,
                            "EE_accel": 0,
                            "time": 0,
                            "Jspeed": copy.deepcopy(zero_Jvel),
                            "Jaccel": copy.deepcopy(zero_Jvel),
                        }
                    )
                    init_transition_indexes.append(len(corrected_traj) - 1)

                    next_dist = self.compute_lin_or_ang_distance(
                        pose, plan_poses[min(j + 1, len(plan_poses) - 1)], linear
                    )
                    if (x_plan + next_dist) > v_change[v_chg_i]["x_min_req"] or (
                        traj_mov[i] < v_change[v_chg_i]["x_min_req"]
                        and j >= len(plan_poses) - 1
                    ):
                        speed_diff = EE_speed_aux[s_i] - EE_speed_aux[s_i - 1]
                        trans_accel = min(
                            (2 * speed_diff * EE_speed_aux[s_i - 1] + speed_diff**2)
                            / (2 * x_plan),
                            max_accel,
                        )
                        for index in init_transition_indexes:
                            corrected_traj[index]["EE_accel"] = trans_accel
                            tA = trans_accel / 2
                            tB = corrected_traj[index - 1]["EE_speed"]
                            tC = -self.compute_lin_or_ang_distance(
                                corrected_traj[index]["pose"],
                                corrected_traj[index - 1]["pose"],
                                linear,
                            )
                            disc = tB**2 - 4 * tA * tC
                            new_time_1 = (-tB + math.sqrt(disc)) / (2 * tA)
                            new_time_2 = (-tB - math.sqrt(disc)) / (2 * tA)
                            if new_time_1 < 0 and new_time_2 < 0:
                                new_time = 0
                                success = False
                            elif new_time_1 < 0:
                                new_time = new_time_2
                            elif new_time_2 < 0:
                                new_time = new_time_1
                            elif new_time_1 < new_time_2:
                                new_time = new_time_1
                            else:
                                new_time = new_time_2
                            corrected_traj[index]["time"] = (
                                corrected_traj[index - 1]["time"] + new_time
                            )
                            corrected_traj[index]["EE_speed"] = (
                                corrected_traj[index - 1]["EE_speed"]
                                + trans_accel * new_time
                            )
                        corrected_traj[-1]["EE_accel"] = 0
                        corrected_traj[init_transition_indexes[0] - 1]["EE_accel"] = (
                            trans_accel
                        )
                        v_chg_i += 1
                        init_speed_change = False

                elif final_speed_change:
                    x_plan += self.compute_lin_or_ang_distance(
                        pose, corrected_traj[-1]["pose"], linear
                    )
                    x_left = traj_mov[i] - x_plan

                    if x_left < v_change[v_chg_i]["x_min_req"]:
                        if first_final:
                            x_trans = x_left + self.compute_lin_or_ang_distance(
                                pose, corrected_traj[-1]["pose"], linear
                            )
                            speed_diff = (
                                EE_speed_aux[s_i + 1] - corrected_traj[-1]["EE_speed"]
                            )
                            ta1 = (
                                2 * speed_diff * EE_speed_aux[s_i] + speed_diff**2
                            ) / (2 * x_trans)
                            ta2 = -max_accel
                            trans_accel = ta2 if abs(ta1) > abs(ta2) else ta1
                            first_final = False
                            corrected_traj[-1]["EE_accel"] = trans_accel

                        if speed_diff == 0:
                            corrected_traj.append(
                                {
                                    "pose": pose,
                                    "state": all_plans[i]
                                    .joint_trajectory.points[j]
                                    .positions,
                                    "EE_speed": 0,
                                    "EE_accel": 0,
                                    "time": corrected_traj[-1]["time"],
                                    "Jspeed": copy.deepcopy(zero_Jvel),
                                    "Jaccel": copy.deepcopy(zero_Jvel),
                                }
                            )
                            continue

                        tA = trans_accel / 2
                        tB = corrected_traj[-1]["EE_speed"]
                        tC = -self.compute_lin_or_ang_distance(
                            pose, corrected_traj[-1]["pose"], linear
                        )
                        disc = tB**2 - 4 * tA * tC
                        if disc <= 0:
                            new_time = (-tB) / (2 * tA)
                        else:
                            new_time_1 = (-tB + math.sqrt(disc)) / (2 * tA)
                            new_time_2 = (-tB - math.sqrt(disc)) / (2 * tA)
                            if new_time_1 < 0 and new_time_2 < 0:
                                new_time = 0
                                success = False
                            elif new_time_1 < 0:
                                new_time = new_time_2
                            elif new_time_2 < 0:
                                new_time = new_time_1
                            elif new_time_1 < new_time_2:
                                new_time = new_time_1
                            else:
                                new_time = new_time_2

                        new_total_time = corrected_traj[-1]["time"] + new_time
                        new_speed = (
                            corrected_traj[-1]["EE_speed"] + trans_accel * new_time
                        )
                        corrected_traj.append(
                            {
                                "pose": pose,
                                "state": all_plans[i]
                                .joint_trajectory.points[j]
                                .positions,
                                "EE_speed": new_speed,
                                "EE_accel": trans_accel,
                                "time": new_total_time,
                                "Jspeed": copy.deepcopy(zero_Jvel),
                                "Jaccel": copy.deepcopy(zero_Jvel),
                            }
                        )
                        if (linear and x_left < 0.1) or (
                            not linear and x_left < 0.07 * (math.pi / 180)
                        ):
                            corrected_traj[-1]["EE_accel"] = 0
                            v_chg_i += 1
                            final_speed_change = False
                    else:
                        corrected_traj.append(
                            {
                                "pose": pose,
                                "state": all_plans[i]
                                .joint_trajectory.points[j]
                                .positions,
                                "EE_speed": EE_speed_aux[s_i],
                                "EE_accel": 0,
                                "time": corrected_traj[-1]["time"]
                                + self.compute_lin_or_ang_distance(
                                    pose, corrected_traj[-1]["pose"], linear
                                )
                                / EE_speed_aux[s_i],
                                "Jspeed": copy.deepcopy(zero_Jvel),
                                "Jaccel": copy.deepcopy(zero_Jvel),
                            }
                        )
                else:
                    corrected_traj.append(
                        {
                            "pose": pose,
                            "state": all_plans[i].joint_trajectory.points[j].positions,
                            "EE_speed": EE_speed_aux[s_i],
                            "EE_accel": 0,
                            "time": corrected_traj[-1]["time"]
                            + self.compute_lin_or_ang_distance(
                                pose, corrected_traj[-1]["pose"], linear
                            )
                            / EE_speed_aux[s_i],
                            "Jspeed": copy.deepcopy(zero_Jvel),
                            "Jaccel": copy.deepcopy(zero_Jvel),
                        }
                    )

            i += 1
            s_i += 1

        return corrected_traj, success

    def _plan_cartesian_path_waypoints(
        self,
        waypoints: list,  # list of geometry_msgs/Pose
        start_joint_positions: list,  # starting joint positions
        joint_names: list,
        step: float = 0.0005,
    ):
        """
        Call the MoveIt2 GetCartesianPath service with all waypoints.
        Returns (RobotTrajectory, fraction) or (None, 0.0) on failure.
        """
        from moveit_msgs.msg import RobotState
        from moveit_msgs.srv import GetCartesianPath

        req = GetCartesianPath.Request()
        req.header.frame_id = self.BASE_LINK
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = self.GROUP_NAME
        req.link_name = self.EEF_LINK
        req.waypoints = waypoints[1:] if len(waypoints) > 1 else waypoints
        req.max_step = step
        req.jump_threshold = 0.0  # 0.0 disables jump check for frames
        req.avoid_collisions = True

        self.get_logger().info(
            f"GetCartesianPath request | waypoints={len(req.waypoints)} | "
            f"max_step={req.max_step} | start_joints={list(req.start_state.joint_state.position)}"
        )

        # Start state for chaining segments
        rs = RobotState()
        rs.joint_state.name = list(joint_names)
        rs.joint_state.position = list(start_joint_positions)
        req.start_state = rs

        self.get_logger().info(
            f"GetCartesianPath request | waypoints={len(req.waypoints)} | "
            f"max_step={req.max_step} | "
            f"start_joints={list(req.start_state.joint_state.position)}"
        )

        future = self._cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()

        if result is None:
            self.get_logger().error("GetCartesianPath: no response from service.")
            return None, 0.0

        if result.fraction < 0.5:
            self.get_logger().error(
                f"GetCartesianPath: fraction too low ({result.fraction:.2f}). "
                "Check that the waypoints are reachable."
            )
            return None, result.fraction

        self.get_logger().info(
            f"GetCartesianPath OK | fraction={result.fraction:.3f} | "
            f"waypoints={len(result.solution.joint_trajectory.points)}"
        )

        # Diagnostic: compare requested max_step vs actual step size
        n_pts = len(result.solution.joint_trajectory.points)
        if n_pts >= 2:
            pose_first = self._fk_from_joint_positions(
                joint_names, list(result.solution.joint_trajectory.points[0].positions)
            )
            pose_last = self._fk_from_joint_positions(
                joint_names, list(result.solution.joint_trajectory.points[-1].positions)
            )
            if pose_first and pose_last:
                cart_dist_mm = self.compute_distance(
                    self._pose_to_mm(pose_first), self._pose_to_mm(pose_last)
                )
                actual_step_mm = cart_dist_mm / (n_pts - 1) if n_pts > 1 else 0
                ratio = actual_step_mm / step if step > 0 else float("inf")
                self.get_logger().info(
                    f"[diag] max_step requested={step*1000:.1f}mm | "
                    f"actual avg step={actual_step_mm:.1f}mm | "
                    f"ratio={ratio:.1f}x | cart_dist={cart_dist_mm:.1f}mm"
                )

        return result.solution, result.fraction

    def verify_cartesian_path(
        self,
        plan,
        joint_names: list,
        tolerance_mm: float = 1.0,
    ) -> bool:
        """
        Verify that the FK waypoints of the plan describe an approximately
        straight-line Cartesian trajectory using PyKDL.
        """
        pts = plan.joint_trajectory.points
        if len(pts) < 2:
            self.get_logger().warn("Plan with fewer than 2 waypoints, cannot verify.")
            return False

        # FK of the first and last waypoint to define the reference line
        pose_start = self._fk_from_joint_positions(joint_names, list(pts[0].positions))
        pose_end = self._fk_from_joint_positions(joint_names, list(pts[-1].positions))

        if pose_start is None or pose_end is None:
            self.get_logger().error("FK failed while verifying the Cartesian path.")
            return False

        f_start = self.pose_to_frame(pose_start)
        f_end = self.pose_to_frame(pose_end)

        # Direction vector of the ideal straight line (in meters)
        p0 = f_start.p
        p1 = f_end.p
        line_vec = p1 - p0
        line_len = line_vec.Norm()

        if line_len < 1e-6:
            self.get_logger().warn(
                "Start and end are the same point — null trajectory."
            )
            return True

        line_dir = line_vec * (1.0 / line_len)

        max_deviation_mm = 0.0
        n_points = len(pts)

        for i, pt in enumerate(pts):
            pose_i = self._fk_from_joint_positions(joint_names, list(pt.positions))
            if pose_i is None:
                continue
            p_i = self.pose_to_frame(pose_i).p

            # Distance from the point to the ideal line p0→p1
            v = p_i - p0
            # Scalar projection onto the direction
            proj_scalar = PyKDL.dot(v, line_dir)
            # Projected point on the line
            proj_point = p0 + line_dir * proj_scalar
            # Perpendicular distance (deviation from the straight path)
            deviation = (p_i - proj_point).Norm() * 1000.0  # a mm

            if deviation > max_deviation_mm:
                max_deviation_mm = deviation

        ok = max_deviation_mm <= tolerance_mm
        status = "✓ OK" if ok else "✗ EXCESSIVE DEVIATION"
        self.get_logger().info(
            f"[verify_cartesian_path] {status} | "
            f"Waypoints: {n_points} | "
            f"Longitud: {line_len * 1000:.2f} mm | "
            f"Max deviation: {max_deviation_mm:.3f} mm (limit: {tolerance_mm} mm)"
        )
        return ok

    def compute_cartesian_path_velocity_control(
        self,
        waypoints_list: list,
        EE_speed: list,
        EE_ang_speed: list = [],
        max_linear_accel: float = 200.0,
        max_ang_accel: float = 140.0,
        extra_info: bool = False,
        step: float = 0.0005,
    ):
        success = True

        # Default angular speed if not specified
        if not EE_ang_speed:
            EE_ang_speed = [s * 0.7 for s in EE_speed]

        # Convertir a rad
        EE_ang_speed = [a * (math.pi / 180) for a in EE_ang_speed]
        max_ang_accel *= math.pi / 180

        # Speed profiles
        EE_speed_aux = [0] + list(EE_speed) + [0]
        EE_ang_speed_aux = [0] + list(EE_ang_speed) + [0]
        v_change = []
        v_change_ang = []

        for i in range(len(EE_speed_aux) - 1):
            a_lin = max_linear_accel * (
                -1 if EE_speed_aux[i] > EE_speed_aux[i + 1] else 1
            )
            a_ang = max_ang_accel * (
                -1 if EE_ang_speed_aux[i] > EE_ang_speed_aux[i + 1] else 1
            )
            t_lin = (EE_speed_aux[i + 1] - EE_speed_aux[i]) / a_lin
            t_ang = (EE_ang_speed_aux[i + 1] - EE_ang_speed_aux[i]) / a_ang
            v_change.append(
                {
                    "t_req": t_lin,
                    "x_min_req": EE_speed_aux[i] * t_lin + (a_lin * t_lin**2) / 2,
                }
            )
            v_change_ang.append(
                {
                    "t_req": t_ang,
                    "x_min_req": EE_ang_speed_aux[i] * t_ang + (a_ang * t_ang**2) / 2,
                }
            )
        all_plans = []
        joint_names = None

        # Initial state: current robot joint position
        current_js = self._moveit2.joint_state
        if current_js is None:
            # Same as in _get_current_eef_pose: no /joint_states message may
            # have arrived yet. Give a second chance before aborting planning.
            self.wait_for_joint_state(timeout_sec=2.0)
            current_js = self._moveit2.joint_state
        if current_js is None:
            self.get_logger().error(
                "No joint_state available to start planning "
                "(no /joint_states messages)."
            )
            return None, False

        # start_positions = list(current_js.position)
        # start_names     = list(current_js.name)

        try:
            self.get_logger().info(
                f"joint_state names: {list(current_js.name)} | "
                f"positions: {list(current_js.position)}"
            )
            js_map = dict(zip(current_js.name, current_js.position))
            start_names = list(self.JOINT_NAMES)
            start_positions = [js_map[n] for n in self.JOINT_NAMES]
        except KeyError as e:
            self.get_logger().error(
                f"Joint {e} no encontrado en joint_state. "
                f"Available names: {list(current_js.name)}"
            )
            return None, False

        for traj in waypoints_list:
            # traj is a list of Pose — all of them are sent to the service
            plan, fraction = self._plan_cartesian_path_waypoints(
                waypoints=list(traj),  # full list, not just the last
                start_joint_positions=start_positions,
                joint_names=start_names,
                step=step,
                #step=0.0005, # for diagnostics, step-by-step for analysis only
            )

            if plan is None:
                self.get_logger().error(
                    f"Segmento cartesiano fallido (fraction={fraction:.2f})."
                )
                return None, False

            all_plans.append(plan)

            if joint_names is None:
                joint_names = list(plan.joint_trajectory.joint_names)

            if not self.verify_cartesian_path(plan, joint_names, tolerance_mm=1.0):
                self.get_logger().warn(
                    "The segment does not follow a straight line — check the waypoints."
                )
                # no need to return anything, but keep it logged

            # Encadenar: el inicio del siguiente segmento es el final de este
            last_pt = plan.joint_trajectory.points[-1]
            start_positions = list(last_pt.positions)
            start_names = list(joint_names)  # names don't change between segments

        # Reset start state to the actual current state
        self._moveit2.clear_goal_constraints()

        # ---- FK for each waypoint of all plans ----
        # Uses /compute_fk (standard MoveIt2 service)
        traj_poses = []
        traj_mov_position = []
        traj_mov_angle = []

        for plan in all_plans:
            plan_poses = []
            traj_mov_i_pos = 0.0
            traj_mov_i_ang = 0.0

            for joint_state_pt in plan.joint_trajectory.points:
                pose_m = self._fk_from_joint_positions(
                    joint_names, list(joint_state_pt.positions)
                )
                if pose_m is None:
                    self.get_logger().error("FK failed at a trajectory waypoint.")
                    return None, False

                pose_mm = copy.deepcopy(pose_m)
                pose_mm.position.x *= 1000
                pose_mm.position.y *= 1000
                pose_mm.position.z *= 1000
                plan_poses.append(pose_mm)

                if len(plan_poses) > 1:
                    traj_mov_i_pos += self.compute_distance(
                        plan_poses[-2], plan_poses[-1]
                    )
                    traj_mov_i_ang += self.compute_angle_distance(
                        plan_poses[-2], plan_poses[-1]
                    )

            traj_poses.append(plan_poses)
            traj_mov_position.append(traj_mov_i_pos)
            traj_mov_angle.append(traj_mov_i_ang)

        # Joint velocity limits from URDF (cached after first retrieval)
        if self._vel_limit is None:
            robot_desc = self._get_robot_description()
            vel_limit = {}
            if robot_desc:
                root = ET.fromstring(robot_desc)
                for child in root:
                    if child.tag == "joint" and child.get("type") == "revolute":
                        j_name = child.get("name")
                        for attrib in child:
                            if attrib.tag == "limit":
                                vel_limit[j_name] = float(attrib.get("velocity")) * 0.9
                self._vel_limit = vel_limit
                self.get_logger().info(
                    f"Joint velocity limits cached: {list(vel_limit.keys())}"
                )
            else:
                self._vel_limit = {}  # empty dict = enforcement disabled
        vel_limit = self._vel_limit

        # Recalculate times with speed profiles
        corrected_traj, success_lin = self.adjust_plan_speed(
            traj_poses,
            EE_speed_aux,
            v_change,
            traj_mov_position,
            max_linear_accel,
            all_plans,
            linear=True,
        )
        corrected_traj_ang, success_ang = self.adjust_plan_speed(
            traj_poses,
            EE_ang_speed_aux,
            v_change_ang,
            traj_mov_angle,
            max_ang_accel,
            all_plans,
            linear=False,
        )
        if not success_lin or not success_ang:
            success = False

        # ---- Merge linear + angular, take the larger dt
        first_accel = True
        t_accel = 0.0
        t_dec = 0.0
        full_corrected_traj = copy.deepcopy(corrected_traj)
        for i in range(len(corrected_traj) - 1):
            dt_lin = corrected_traj[i + 1]["time"] - corrected_traj[i]["time"]
            dt_ang = corrected_traj_ang[i + 1]["time"] - corrected_traj_ang[i]["time"]
            full_corrected_traj[i + 1]["time"] = full_corrected_traj[i]["time"] + max(
                dt_lin, dt_ang
            )
            if extra_info:
                if (
                    corrected_traj[i + 1]["EE_accel"] == 0
                    and corrected_traj_ang[i + 1]["EE_accel"] == 0
                ):
                    if first_accel:
                        first_accel = False
                        t_accel = full_corrected_traj[i + 1]["time"]
                    if (i + 1) < (len(corrected_traj) - 1):
                        t_dec = full_corrected_traj[i + 1]["time"]

        # Enforcement of joint limits
        n_joints = len(all_plans[0].joint_trajectory.points[0].positions)
        zero_Jvel = [0.0] * n_joints
        full_corrected_traj_with_limits = copy.deepcopy(full_corrected_traj)

        for i in range(len(full_corrected_traj) - 1):
            time_diff = (
                full_corrected_traj[i + 1]["time"] - full_corrected_traj[i]["time"]
            )

            # Guard: if time_diff is 0 or negative, copy state from the
            # previous waypoint and force a minimum dt of 1ms to preserve monotonicity.
            if time_diff <= 0:
                if time_diff == 0:
                    self.get_logger().warn(
                        f"Waypoints {i+1} and {i} share the same timestamp — "
                        "copying state from previous waypoint."
                    )
                full_corrected_traj_with_limits[i + 1]["Jspeed"] = copy.deepcopy(
                    full_corrected_traj_with_limits[i]["Jspeed"]
                )
                full_corrected_traj_with_limits[i + 1]["Jaccel"] = copy.deepcopy(
                    full_corrected_traj_with_limits[i]["Jaccel"]
                )
                full_corrected_traj_with_limits[i + 1]["time"] = (
                    full_corrected_traj_with_limits[i]["time"] + 0.001
                )
                continue

            updated_new_times = []
            update_time = False

            for j in range(n_joints):
                angle_diff = (
                    full_corrected_traj[i + 1]["state"][j]
                    - full_corrected_traj[i]["state"][j]
                )

                # Guard: no joint movement -> inherit velocities
                if abs(angle_diff) < 1e-12:
                    full_corrected_traj_with_limits[i + 1]["Jspeed"][j] = (
                        full_corrected_traj_with_limits[i]["Jspeed"][j]
                    )
                    full_corrected_traj_with_limits[i + 1]["Jaccel"][j] = 0.0
                    continue

                new_Jaccel = (
                    angle_diff
                    - full_corrected_traj_with_limits[i]["Jspeed"][j] * time_diff
                ) * (2 / time_diff**2)
                new_Jspeed = (
                    full_corrected_traj_with_limits[i]["Jspeed"][j]
                    + new_Jaccel * time_diff
                )

                # Lookup joint limit using joint_names (not rs.joint_state.name)
                j_name = (
                    joint_names[j] if (joint_names and j < len(joint_names)) else ""
                )
                limit = vel_limit.get(j_name, float("inf"))

                if abs(new_Jspeed) > limit:
                    new_Jspeed = math.copysign(limit, new_Jspeed)
                    if abs(angle_diff) > 1e-12:
                        new_Jaccel = (
                            2
                            * full_corrected_traj_with_limits[i]["Jspeed"][j]
                            * (
                                new_Jspeed
                                - full_corrected_traj_with_limits[i]["Jspeed"][j]
                            )
                            + (
                                new_Jspeed
                                - full_corrected_traj_with_limits[i]["Jspeed"][j]
                            )
                            ** 2
                        ) / (2 * angle_diff)
                    else:
                        new_Jaccel = 0.0
                    new_time_step = (
                        (angle_diff / new_Jspeed)
                        if abs(new_Jaccel) < 0.0001
                        else (
                            new_Jspeed
                            - full_corrected_traj_with_limits[i]["Jspeed"][j]
                        )
                        / new_Jaccel
                    )
                    if new_time_step > 0:
                        updated_new_times.append(new_time_step)
                        update_time = True

                full_corrected_traj_with_limits[i + 1]["Jaccel"][j] = new_Jaccel
                full_corrected_traj_with_limits[i + 1]["Jspeed"][j] = new_Jspeed

            full_corrected_traj_with_limits[i + 1]["time"] = (
                full_corrected_traj_with_limits[i]["time"] + time_diff
            )

            if update_time and updated_new_times:
                self.get_logger().warn(
                    "Joint velocity limit exceeded — rescaling time."
                )
                new_time_diff = max(updated_new_times)
                if new_time_diff <= 0:
                    continue
                full_corrected_traj_with_limits[i + 1]["time"] = (
                    full_corrected_traj_with_limits[i]["time"] + new_time_diff
                )
                for j in range(n_joints):
                    angle_diff = (
                        full_corrected_traj[i + 1]["state"][j]
                        - full_corrected_traj[i]["state"][j]
                    )
                    if abs(angle_diff) < 1e-12:
                        continue
                    new_Jaccel = (
                        angle_diff
                        - full_corrected_traj_with_limits[i]["Jspeed"][j]
                        * new_time_diff
                    ) * (2 / new_time_diff**2)
                    new_Jspeed = (
                        full_corrected_traj_with_limits[i]["Jspeed"][j]
                        + new_Jaccel * new_time_diff
                    )
                    full_corrected_traj_with_limits[i + 1]["Jaccel"][j] = new_Jaccel
                    full_corrected_traj_with_limits[i + 1]["Jspeed"][j] = new_Jspeed

            if extra_info and t_accel == full_corrected_traj[i + 1]["time"]:
                t_accel = full_corrected_traj_with_limits[i + 1]["time"]
            if extra_info and t_dec == full_corrected_traj[i + 1]["time"]:
                t_dec = full_corrected_traj_with_limits[i + 1]["time"]

        full_corrected_traj_with_limits[-1]["Jspeed"] = copy.deepcopy(zero_Jvel)

        # Construir el mensaje RobotTrajectory
        new_plan = RobotTrajectory()
        new_plan.joint_trajectory.header.frame_id = self.BASE_LINK
        new_plan.joint_trajectory.joint_names = list(joint_names) if joint_names else []

        for state in full_corrected_traj_with_limits:
            point = JointTrajectoryPoint()
            point.positions = list(state["state"])
            point.velocities = list(state["Jspeed"])
            point.accelerations = list(state["Jaccel"])
            point.effort = []
            t_secs = int(state["time"])
            t_nsecs = int((state["time"] - t_secs) * 1_000_000_000)
            point.time_from_start = Duration(sec=t_secs, nanosec=t_nsecs)
            new_plan.joint_trajectory.points.append(point)

        if extra_info:
            return new_plan, success, t_accel, t_dec
        return new_plan, success


def main(args=None):
    move_speed = ask_speed(default=50.0)
    print(
        f"-> Approach and retraction at {move_speed:.1f} Cartesian mm/s.\n"
    )

    rclpy.init(args=args)
    control = MoveGroupPythonIntefaceControl()
    control.get_logger().info(
        f"Cartesian speed (approach/retraction): "
        f"{move_speed:.1f} mm/s"
    )

    control.get_logger().info("Waiting for first /joint_states message…")
    if not control.wait_for_joint_state(timeout_sec=10.0):
        control.get_logger().error(
            "No /joint_states message received after 10s. "
            "Check that the robot driver/simulation is publishing "
            "that topic and the namespace ('/lbr') is correct."
        )
        control.destroy_node()
        rclpy.shutdown()
        return
    control.get_logger().info("joint_state received. Starting demo.")

    try:
        offset = 0.1

        # All known sponges — each has its own Z offset and slight
        # orientation adjustments (roll/pitch) for the experiment setup.
        SPONGES = [
            {"name": "green",  "R": -math.pi,              "P": -4.15,  "z0": 0.3651},
            {"name": "yellow", "R": -math.pi - 0.2,         "P": -4.10,  "z0": 0.3644},
            {"name": "orange", "R": -math.pi + 0.2,         "P": -4.25,  "z0": 0.3642},
            {"name": "blue",   "R": -math.pi + 0.2,         "P": -3.95,  "z0": 0.3645},
            {"name": "red",    "R": -math.pi,              "P": -4.30,  "z0": 0.3648},
        ]
        # Convert P from degrees to radians
        for s in SPONGES:
            s["P"] *= math.pi / 180.0
            # Y is always pi/2 for all sponges
            s["Y"] = math.pi / 2.0

        print("\nAvailable sponges:")
        for i, s in enumerate(SPONGES):
            print(f"  [{i + 1}] {s['name']}  (z0={s['z0']:.4f} m, P={s['P'] * 180.0 / math.pi:.2f}°)")
        print(f"  [a] all")

        choice = input("Select sponge [Enter = green]: ").strip().lower()
        if choice == "a":
            selected_sponges = SPONGES
        elif choice == "":
            selected_sponges = [SPONGES[0]]  # default: green
        else:
            try:
                idx = int(choice) - 1
                if 0 <= idx < len(SPONGES):
                    selected_sponges = [SPONGES[idx]]
                else:
                    print(f"  -> Invalid index, using green.")
                    selected_sponges = [SPONGES[0]]
            except ValueError:
                print(f"  -> Invalid choice, using green.")
                selected_sponges = [SPONGES[0]]

        x0 = 0.0
        y0 = 0.543

        for sponge in selected_sponges:
            R = sponge["R"]
            P = sponge["P"]
            Y = sponge["Y"]
            z0 = sponge["z0"]

            control.get_logger().info(
                f"Selected sponge: {sponge['name']} "
                f"(z0={z0:.4f} m, R={R:.4f} rad, P={P * 180.0 / math.pi:.2f}°)"
            )

            quat = quaternion_from_euler(R, P, Y)
            q_norm = math.sqrt(sum(q**2 for q in quat))

            target = Pose()
            target.orientation.x = quat[0] / q_norm
            target.orientation.y = quat[1] / q_norm
            target.orientation.z = quat[2] / q_norm
            target.orientation.w = quat[3] / q_norm

            # Move to approach pose, now with Cartesian speed control
            # (instead of go_to_pose "free", which lets MoveIt choose the timing)
            target.position.x = x0
            target.position.y = y0
            target.position.z = z0 + offset
            if not control.go_to_pose_speed(target, move_speed, accel=100.0):
                control.get_logger().error(
                    "Could not reach initial approach pose "
                    f"for sponge '{sponge['name']}'. "
                    "Skipping to next sponge."
                )
                continue

            control.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))

            penetrations = [0.001, 0.003, 0.005, 0.007]  # m
            # Hysteresis test speeds: this is the experiment's own
            # variable, not asked from the user via console.
            hysteresis_speeds = [70.0, 50.0, 30.0, 10.0]  # mm/s

            abort_demo = False

            # Hysteresis test
            for p in penetrations:
                if abort_demo:
                    break

                control.get_logger().info(
                    f"Sponge '{sponge['name']}' | Penetration: {p} m"
                )

                for s in hysteresis_speeds:
                    control.get_logger().info(
                        f"Sponge '{sponge['name']}' | Penetration speed (test): {s} mm/s"
                    )
                    control.reset_force(settle_time=0.5)

                    target.position.x = x0
                    target.position.y = y0
                    target.position.z = z0 - p
                    ok = control.go_to_pose_speed(target, s, accel=2000.0)
                    if not ok:
                        control.get_logger().error(
                            f"Penetration move at {s} mm/s failed "
                            f"(sponge '{sponge['name']}', p={p} m). "
                            "Skipping this combination and "
                            "continuing with the next test speed."
                        )
                        continue

                    # Penetration time
                    # control.get_clock().sleep_for(rclpy.duration.Duration(seconds=100))
                    control.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))

                    target.position.x = x0
                    target.position.y = y0
                    target.position.z = z0 + offset
                    ok = control.go_to_pose_speed(target, move_speed, accel=100.0)
                    if not ok:
                        control.get_logger().error(
                            "Failed to retract EEF after penetration "
                            f"(sponge '{sponge['name']}', p={p} m, "
                            f"s={s} mm/s). Stopping demo for "
                            "safety: continuing could attempt another "
                            "penetration from an unknown pose."
                        )
                        abort_demo = True
                        break

                    # Rest time
                    # control.get_clock().sleep_for(rclpy.duration.Duration(seconds=400))
                    control.get_clock().sleep_for(rclpy.duration.Duration(seconds=2))

            if abort_demo:
                break

    except KeyboardInterrupt:
        pass
    finally:
        control.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()