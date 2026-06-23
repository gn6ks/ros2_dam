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
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from sensor_msgs.msg import JointState

# pymoveit2: stable wrapper around the MoveIt2 API
from pymoveit2 import MoveIt2

from rclpy.node import Node
from tf_transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectoryPoint

FT_TOPIC_CANDIDATES = [
    "/ft_sensor/wrench",
    "/lbr/ft_sensor/wrench",
]

WINDOW_SIZE = 5


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


def all_close(goal, actual, tolerance):
    """Compara dos poses o listas con tolerancia."""
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
    """Pregunta al usuario la velocidad cartesiana del EEF (mm/s)."""
    while True:
        raw = input(f"Cartesian EEF speed in mm/s [Enter = {default}]: ").strip()
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
                    f"  -> {speed} mm/s is high; MoveIt2 may reject the plan. Continue? [y/N]: "
                )
                .strip()
                .lower()
            )
            if confirm != "s":
                continue
        return speed

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

        # ── pymoveit2: wrapper fino sobre MoveIt2 ──
        self._moveit2 = MoveIt2(
            node=self,
            joint_names=self.JOINT_NAMES,
            base_link_name=self.BASE_LINK,
            end_effector_name=self.EEF_LINK,
            group_name=self.GROUP_NAME,
        )

        # Publicador para RViz2
        self._display_traj_pub = self.create_publisher(
            DisplayTrajectory, "display_planned_path", 20
        )

        self.eef_link = self.EEF_LINK

        # ── Caché de límites de velocidad articular (se llena bajo demanda) ──
        self._vel_limit: dict | None = None

        # ── FT sensor ──
        self._wrench_lock = threading.Lock()
        self._active_ft_topic: str | None = None
        self._ft_sub = None
        self._fx_window: deque = deque(maxlen=WINDOW_SIZE)
        self._fy_window: deque = deque(maxlen=WINDOW_SIZE)
        self._fz_window: deque = deque(maxlen=WINDOW_SIZE)
        self._discover_ft_topic()
        if self._active_ft_topic is None:
            self.get_logger().warn("No FT sensor topic found.")

        self.get_logger().info("MoveGroupPythonIntefaceControl ready.")

    def _discover_ft_topic(self, timeout_per_topic: float = 2.0):
        self.get_logger().info("Searching FT sensor topic…")
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
                self.get_logger().info(f"  -> FT topic active: {topic}")
                self.destroy_subscription(sub)
                self._ft_sub = self.create_subscription(
                    WrenchStamped, topic, self._wrench_cb, 10
                )
                return
            self.destroy_subscription(sub)
        self.get_logger().warn("No FT sensor publishing data.")

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
            self.get_logger().warn("reset_force: no FT topic available.")
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
                f"reset_force: window not filled after {settle_time:.2f}s"
            )
        return filled

    def wait_for_joint_state(self, timeout_sec: float = 10.0) -> bool:
        """Espera hasta que llegue al menos un mensaje de /joint_states."""
        start = self.get_clock().now()
        while self._moveit2.joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = (self.get_clock().now() - start).nanoseconds * 1e-9
            if elapsed > timeout_sec:
                return False
        return True

    def _get_current_eef_pose(self) -> Pose:
        """Devuelve la pose actual del EEF via FK."""
        js = self._moveit2.joint_state
        if js is None:
            self.wait_for_joint_state(timeout_sec=2.0)
            js = self._moveit2.joint_state
        if js is None:
            self.get_logger().error("No joint_state available.")
            return Pose()
        # compute_fk espera JointState; si se omite usa el estado actual
        fk_result = self._moveit2.compute_fk(joint_state=js)
        if fk_result is None:
            self.get_logger().error("FK failed for current state.")
            return Pose()
        return fk_result.pose

    def go_to_joint_state(self):
        """Mueve a la configuración articular fija del benchmark."""
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
        """Planifica y ejecuta un movimiento cartesiano sin control de velocidad."""
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
            self.get_logger().error(f"go_to_pose failed: {exc!r}")
            return False
        return True

    def _plan_and_execute(self, trajectory, success: bool = True) -> bool:
        """Publica el plan en RViz2 y lo ejecuta via pymoveit2.execute()."""
        if trajectory is None:
            self.get_logger().error("Null trajectory — cannot execute.")
            return False

        # Publicar en RViz2
        try:
            display_traj = DisplayTrajectory()
            display_traj.trajectory.append(trajectory)
            self._display_traj_pub.publish(display_traj)
            self.get_logger().info("Plan published to RViz2.")
        except Exception as exc:
            self.get_logger().warn(f"Could not publish to RViz: {exc}")

        if not success:
            self.get_logger().warn(
                "Speed calculation had warnings; executing anyway."
            )

        # pymoveit2.execute() gestiona FollowJointTrajectory internamente
        try:
            self._moveit2.execute(trajectory)
            self._moveit2.wait_until_executed()
            self.get_logger().info("Trajectory executed successfully.")
            return True
        except Exception as exc:
            self.get_logger().error(f"Execution error: {exc!r}")
            return False

    def follow_trajectory_speed(
        self, waypoints, speeds, ang_speeds=None, accel=100.0
    ) -> bool:
        """Ejecuta una trayectoria multi-segmento con control de velocidad."""
        if ang_speeds is None:
            ang_speeds = []
        try:
            trajectory, success = self.compute_cartesian_path_velocity_control(
                waypoints, speeds, EE_ang_speed=ang_speeds, max_linear_accel=accel
            )
        except Exception as exc:
            self.get_logger().error(f"Error planning multi-section trajectory: {exc!r}")
            return False

        if trajectory is None:
            self.get_logger().error("Could not plan multi-section trajectory.")
            return False

        executed = self._plan_and_execute(trajectory, success)
        time.sleep(0.5)
        return executed

    def go_to_pose_speed(
        self, pose: Pose, speed=10.0, ang_speed=None, accel=100.0
    ) -> bool:
        """Movimiento cartesiano a una velocidad dada (mm/s)."""
        if ang_speed is None:
            ang_speed = []
        try:
            current_pose = self._get_current_eef_pose()
            waypoints = [[copy.deepcopy(current_pose), copy.deepcopy(pose)]]
            trajectory, success = self.compute_cartesian_path_velocity_control(
                waypoints, [speed], EE_ang_speed=ang_speed, max_linear_accel=accel
            )
        except Exception as exc:
            self.get_logger().error(f"Error planning at {speed} mm/s: {exc!r}")
            return False

        if trajectory is None:
            self.get_logger().error(f"Could not plan move at {speed} mm/s.")
            return False

        if not success:
            self.get_logger().warn(
                f"Plan at {speed} mm/s has warnings; executing anyway."
            )

        executed = self._plan_and_execute(trajectory, success)
        time.sleep(0.5)
        return executed

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
            rot[0, 0], rot[1, 0], rot[2, 0],
            rot[0, 1], rot[1, 1], rot[2, 1],
            rot[0, 2], rot[1, 2], rot[2, 2],
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
        if linear:
            return self.compute_distance(pose1, pose2)
        return self.compute_angle_distance(pose1, pose2)

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
                frac = float(point) / float(n_points)
                x = initial_pose.position.x + (final_pose.position.x - initial_pose.position.x) * frac
                y = initial_pose.position.y + (final_pose.position.y - initial_pose.position.y) * frac
                z = initial_pose.position.z + (final_pose.position.z - initial_pose.position.z) * frac
                rotation = self.pose_to_frame(initial_pose).M
                rotation.DoRotX(rad_dif[0] * frac)
                rotation.DoRotY(rad_dif[1] * frac)
                rotation.DoRotZ(rad_dif[2] * frac)
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
        speed_diff = 0.0
        trans_accel = 0.0

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

                # ── aceleración al inicio del segmento ──
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

                # ── deceleración al final del segmento ──
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

                # ── sin cambio de velocidad ──
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

    def compute_cartesian_path_velocity_control(
        self,
        waypoints_list: list,
        EE_speed: list,
        EE_ang_speed: list = None,
        max_linear_accel: float = 200.0,
        max_ang_accel: float = 140.0,
        extra_info: bool = False,
        step: float = 0.0005,
    ):
        """
        Planifica una trayectoria cartesiana multi-segmento con control de
        velocidad del EEF.

        Usa pymoveit2.plan(cartesian=True) para los segmentos y
        pymoveit2.compute_fk() para la cinemática directa.
        """
        if EE_ang_speed is None:
            EE_ang_speed = []
        success = True

        # Velocidades angulares por defecto
        if not EE_ang_speed:
            EE_ang_speed = [s * 0.7 for s in EE_speed]

        # Convertir a rad
        EE_ang_speed = [a * (math.pi / 180) for a in EE_ang_speed]
        max_ang_accel *= math.pi / 180

        # Perfiles de velocidad con aceleración nula en extremos
        EE_speed_aux = [0] + list(EE_speed) + [0]
        EE_ang_speed_aux = [0] + list(EE_ang_speed) + [0]
        v_change = []
        v_change_ang = []

        for i in range(len(EE_speed_aux) - 1):
            a_lin = max_linear_accel * (-1 if EE_speed_aux[i] > EE_speed_aux[i + 1] else 1)
            a_ang = max_ang_accel * (-1 if EE_ang_speed_aux[i] > EE_ang_speed_aux[i + 1] else 1)
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

        # ── 1. Planificar segmentos cartesianos con pymoveit2.plan(cartesian=True) ──
        all_plans = []
        joint_names = None

        current_js = self._moveit2.joint_state
        if current_js is None:
            self.wait_for_joint_state(timeout_sec=2.0)
            current_js = self._moveit2.joint_state
        if current_js is None:
            self.get_logger().error("No joint_state available for planning.")
            return None, False

        for traj in waypoints_list:
            # traj es una lista de Poses; el último elemento es el objetivo
            target = traj[-1]
            try:
                trajectory = self._moveit2.plan(
                    position=[target.position.x, target.position.y, target.position.z],
                    quat_xyzw=[
                        target.orientation.x,
                        target.orientation.y,
                        target.orientation.z,
                        target.orientation.w,
                    ],
                    cartesian=True,
                )
            except Exception as exc:
                self.get_logger().error(f"Cartesian planning failed: {exc!r}")
                return None, False

            if trajectory is None:
                self.get_logger().error("Cartesian plan returned None.")
                return None, False

            all_plans.append(trajectory)

            if joint_names is None:
                joint_names = list(trajectory.joint_trajectory.joint_names)

        self._moveit2.clear_goal_constraints()

        # ── 2. FK para cada waypoint con pymoveit2.compute_fk() ──
        traj_poses = []
        traj_mov_position = []
        traj_mov_angle = []

        for plan in all_plans:
            plan_poses = []
            traj_mov_i_pos = 0.0
            traj_mov_i_ang = 0.0

            for joint_state_pt in plan.joint_trajectory.points:
                js_msg = JointState()
                js_msg.name = list(joint_names) if joint_names else self.JOINT_NAMES
                js_msg.position = list(joint_state_pt.positions)
                fk_result = self._moveit2.compute_fk(
                    joint_state=js_msg
                )
                if fk_result is None:
                    self.get_logger().error("FK failed at a trajectory waypoint.")
                    return None, False
                pose_m = fk_result.pose

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

        # ── 3. Límites de velocidad articular (desde URDF, bajo demanda) ──
        if self._vel_limit is None:
            self._vel_limit = self._load_vel_limits(
                joint_names if joint_names else self.JOINT_NAMES
            )
        vel_limit = self._vel_limit

        # ── 4. Ajustar velocidades (linear + angular) ──
        corrected_traj, success_lin = self.adjust_plan_speed(
            traj_poses, EE_speed_aux, v_change, traj_mov_position,
            max_linear_accel, all_plans, linear=True,
        )
        corrected_traj_ang, success_ang = self.adjust_plan_speed(
            traj_poses, EE_ang_speed_aux, v_change_ang, traj_mov_angle,
            max_ang_accel, all_plans, linear=False,
        )
        if not success_lin or not success_ang:
            success = False

        # ── 5. Fusionar correcciones lineales + angulares (tomar el dt mayor) ──
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

        # ── 6. Aplicar límites articulares ──
        n_joints = len(all_plans[0].joint_trajectory.points[0].positions)
        zero_Jvel = [0.0] * n_joints
        full_corrected_traj_with_limits = copy.deepcopy(full_corrected_traj)

        for i in range(len(full_corrected_traj) - 1):
            time_diff = (
                full_corrected_traj[i + 1]["time"] - full_corrected_traj[i]["time"]
            )

            if time_diff <= 0:
                if time_diff == 0:
                    self.get_logger().warn(
                        f"Waypoints {i+1} and {i} share timestamp — copying state."
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

                j_name = joint_names[j] if (joint_names and j < len(joint_names)) else ""
                limit = vel_limit.get(j_name, float("inf"))

                if abs(new_Jspeed) > limit:
                    new_Jspeed = math.copysign(limit, new_Jspeed)
                    if abs(angle_diff) > 1e-12:
                        new_Jaccel = (
                            2
                            * full_corrected_traj_with_limits[i]["Jspeed"][j]
                            * (new_Jspeed - full_corrected_traj_with_limits[i]["Jspeed"][j])
                            + (new_Jspeed - full_corrected_traj_with_limits[i]["Jspeed"][j]) ** 2
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
                self.get_logger().warn("Joint velocity limit exceeded — rescaling time.")
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
                        - full_corrected_traj_with_limits[i]["Jspeed"][j] * new_time_diff
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

        # ── 7. Construir RobotTrajectory final ──
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

    def _load_vel_limits(self, joint_names: list) -> dict:
        """
        Carga los límites de velocidad articular (90 % del URDF).
        Intenta varias fuentes en orden: parámetro local, servicio de
        parámetros de move_group, robot_state_publisher. Si ninguna
        funciona, devuelve dict vacío (sin enforce de límites).
        """
        # 1. Intentar parámetro declarado localmente
        try:
            self.declare_parameter("robot_description", "")
            desc = self.get_parameter("robot_description").value
            if desc:
                return self._parse_vel_limits(desc, joint_names)
        except Exception:
            pass

        # 2. Intentar servicio GetParameters de move_group
        from rcl_interfaces.srv import GetParameters
        for srv_name in [
            "/lbr/move_group/get_parameters",
            "/lbr/robot_state_publisher/get_parameters",
        ]:
            try:
                client = self.create_client(GetParameters, srv_name)
                if not client.wait_for_service(timeout_sec=1.0):
                    continue
                req = GetParameters.Request()
                req.names = ["robot_description"]
                future = client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                result = future.result()
                if result and result.values:
                    desc = result.values[0].string_value
                    if desc:
                        self.get_logger().info(
                            f"robot_description retrieved via {srv_name}."
                        )
                        return self._parse_vel_limits(desc, joint_names)
            except Exception:
                continue

        self.get_logger().warn(
            "Could not retrieve robot_description; "
            "joint velocity limits will NOT be enforced."
        )
        return {}

    @staticmethod
    def _parse_vel_limits(urdf_string: str, joint_names: list) -> dict:
        """Extrae los límites de velocidad de un URDF como string."""
        vel_limit = {}
        try:
            root = ET.fromstring(urdf_string)
            for child in root:
                if child.tag == "joint" and child.get("type") == "revolute":
                    j_name = child.get("name")
                    if j_name not in joint_names:
                        continue
                    for attrib in child:
                        if attrib.tag == "limit":
                            vel = float(attrib.get("velocity", 1.0))
                            vel_limit[j_name] = vel * 0.9
                            break
        except ET.ParseError:
            pass
        return vel_limit

def main(args=None):
    move_speed = ask_speed(default=50.0)
    print(f"-> Approach and retraction at {move_speed:.1f} Cartesian mm/s.\n")

    rclpy.init(args=args)
    control = MoveGroupPythonIntefaceControl()
    control.get_logger().info(f"Cartesian speed (approach/retraction): {move_speed:.1f} mm/s")

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

        SPONGES = [
            {"name": "green",  "R": -math.pi,           "P": -4.15,  "z0": 0.3651},
            {"name": "yellow", "R": -math.pi - 0.2,      "P": -4.10,  "z0": 0.3644},
            {"name": "orange", "R": -math.pi + 0.2,      "P": -4.25,  "z0": 0.3642},
            {"name": "blue",   "R": -math.pi + 0.2,      "P": -3.95,  "z0": 0.3645},
            {"name": "red",    "R": -math.pi,           "P": -4.30,  "z0": 0.3648},
        ]
        for s in SPONGES:
            s["P"] *= math.pi / 180.0
            s["Y"] = math.pi / 2.0

        print("\nAvailable sponges:")
        for i, s in enumerate(SPONGES):
            print(
                f"  [{i + 1}] {s['name']}  "
                f"(z0={s['z0']:.4f} m, P={s['P'] * 180.0 / math.pi:.2f}°)"
            )
        print("  [a] all")

        choice = input("Select sponge [Enter = green]: ").strip().lower()
        if choice == "a":
            selected_sponges = SPONGES
        elif choice == "":
            selected_sponges = [SPONGES[0]]
        else:
            try:
                idx = int(choice) - 1
                if 0 <= idx < len(SPONGES):
                    selected_sponges = [SPONGES[idx]]
                else:
                    print("  -> Invalid index, using green.")
                    selected_sponges = [SPONGES[0]]
            except ValueError:
                print("  -> Invalid choice, using green.")
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

            # Approach pose con control de velocidad
            target.position.x = x0
            target.position.y = y0
            target.position.z = z0 + offset
            if not control.go_to_pose_speed(target, move_speed, accel=100.0):
                control.get_logger().error(
                    f"Could not reach initial approach pose "
                    f"for sponge '{sponge['name']}'. Skipping."
                )
                continue

            control.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))

            penetrations = [0.001, 0.003, 0.005, 0.007]  # m
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
                            "Skipping this combination."
                        )
                        continue

                    # Penetration dwell
                    control.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))

                    # Retraction
                    target.position.x = x0
                    target.position.y = y0
                    target.position.z = z0 + offset
                    ok = control.go_to_pose_speed(target, move_speed, accel=100.0)
                    if not ok:
                        control.get_logger().error(
                            "Failed to retract EEF after penetration "
                            f"(sponge '{sponge['name']}', p={p} m, "
                            f"s={s} mm/s). Stopping demo for safety."
                        )
                        abort_demo = True
                        break

                    # Rest time
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
