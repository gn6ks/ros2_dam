#!/usr/bin/env python3

import copy
import math
import sys
import time
import xml.etree.ElementTree as ET

import PyKDL
import rclpy
import rclpy.duration
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.msg import DisplayTrajectory, RobotState, RobotTrajectory

# pymoveit2: biblioteca que envuelve la API de MoveIt2 de forma estable
from pymoveit2 import MoveIt2

# from pymoveit2.robots import (
#     iiwa7 as robot_config,  # cambia al módulo de tu robot si es otro
# )
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Header
from tf_transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectoryPoint


def reset(node: Node, req: bool) -> bool:
    node.get_logger().info("Calibrando sensor FT…")
    # from ft17_publisher.srv import Reset
    # client = node.create_client(Reset, 'FT_reset')
    # if not client.wait_for_service(timeout_sec=5.0):
    #     node.get_logger().error("FT_reset service not available")
    #     return False
    # future = client.call_async(Reset.Request())
    # rclpy.spin_until_future_complete(node, future)
    # return future.result().res
    raise NotImplementedError("Adaptar el tipo Reset del port ROS2 de ft17_publisher")


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
    # GROUP_NAME = "manipulator"
    # BASE_FRAME = "lbr_link_0"  # ajustar al frame base de tu robot
    # EEF_LINK = robot_config.end_effector_name()  # o definir directamente como string

    def __init__(self):
        super().__init__("move_group_control", namespace="/lbr")

        # llamada con pymoveit2
        self._moveit2 = MoveIt2(
            node=self,
            joint_names=self.JOINT_NAMES,
            base_link_name=self.BASE_LINK,
            end_effector_name=self.EEF_LINK,
            group_name=self.GROUP_NAME,
        )

        # Publisher de trayectoria para RViz2
        self._display_traj_pub = self.create_publisher(
            DisplayTrajectory, "display_planned_path", 20
        )

        # Nombre del link end-effector (para FK)
        self.eef_link = self.EEF_LINK

        # Objeto de planning scene (para leer URDF y hacer FK)
        # pymoveit2 expone el robot_model a través de MoveIt2.robot_model
        self.box_name = ""
        self.get_logger().info("MoveGroupPythonIntefaceControl listo.")

    def go_to_joint_state(self):
        """Mueve el robot a la configuración articular fija del benchmark."""
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
        """Planifica y ejecuta movimiento a una pose Cartesiana."""
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
        return True

    def follow_trajectory_speed(self, waypoints, speeds, ang_speeds=[], accel=100.0):
        plan, success = self.compute_cartesian_path_velocity_control(
            waypoints, speeds, EE_ang_speed=ang_speeds, max_linear_accel=accel
        )
        self._publish_and_execute(plan, success)
        time.sleep(0.5)

    def go_to_pose_speed(self, pose: Pose, speed=10.0, ang_speed=[], accel=100.0):
        # Obtener pose actual del EEF
        current_pose = self._get_current_eef_pose()
        waypoints = [[copy.deepcopy(current_pose), copy.deepcopy(pose)]]
        plan, success = self.compute_cartesian_path_velocity_control(
            waypoints, [speed], EE_ang_speed=ang_speed, max_linear_accel=accel
        )
        self._publish_and_execute(plan, success)
        time.sleep(0.5)

    def _publish_and_execute(self, plan, success: bool):
        if plan:
            display_trajectory = DisplayTrajectory()
            display_trajectory.trajectory.append(plan)
            self._display_traj_pub.publish(display_trajectory)
            self.get_logger().info("Plan publicado en RViz2.")
        else:
            self.get_logger().error("No se pudo calcular o publicar el plan.")
            return

        if success:
            # pymoveit2: ejecutar una RobotTrajectory ya construida
            self._moveit2.execute(plan)
            self._moveit2.wait_until_executed()

    def _get_current_eef_pose(self) -> Pose:
        """Devuelve la pose actual del EEF via FK sobre el joint_state actual."""
        js = self._moveit2.joint_state
        if js is None:
            self.get_logger().error("No hay joint_state disponible.")
            return Pose()
        pose = self._fk_from_joint_positions(list(js.name), list(js.position))
        if pose is None:
            self.get_logger().error("FK falló para el estado actual.")
            return Pose()
        return pose

    def _fk_from_joint_positions(
        self, joint_names: list, joint_positions: list
    ) -> Pose | None:
        """
        FK para un conjunto de posiciones articulares.

        pymoveit2 no expone FK directamente, pero podemos usar el servicio
        /compute_fk de MoveIt2 a través de rclpy.
        Este método reemplaza la llamada al servicio iiwa_fk_server del original.
        """
        from moveit_msgs.srv import GetPositionFK
        from std_msgs.msg import Header as StdHeader

        if not hasattr(self, "_fk_client"):
            self._fk_client = self.create_client(GetPositionFK, "/lbr/compute_fk")
            if not self._fk_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("Servicio /compute_fk no disponible.")
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
            self.get_logger().warn("No se puede interpolar: misma pose.")
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
        speed_diff = 0  # FIX: inicializar para evitar UnboundLocalError

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

    def compute_cartesian_path_velocity_control(
        self,
        waypoints_list: list,
        EE_speed: list,
        EE_ang_speed: list = [],
        max_linear_accel: float = 200.0,
        max_ang_accel: float = 140.0,
        extra_info: bool = False,
        step: float = 0.002,
    ):
        success = True

        # Velocidad angular por defecto si no se especifica
        if not EE_ang_speed:
            EE_ang_speed = [s * 0.7 for s in EE_speed]

        # Convertir a rad
        EE_ang_speed = [a * (math.pi / 180) for a in EE_ang_speed]
        max_ang_accel *= math.pi / 180

        # Perfiles de velocidad
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

        # ---- Planificación Cartesiana via pymoveit2 ----
        # pymoveit2 expone compute_cartesian_path() que devuelve RobotTrajectory | None
        all_plans = []
        joint_names = None

        for traj in waypoints_list:
            # Construir lista de poses para pymoveit2
            positions = [[p.position.x, p.position.y, p.position.z] for p in traj]
            quats = [
                [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
                for p in traj
            ]

            self._moveit2.set_pose_goal(
                position=positions[-1],
                quat_xyzw=quats[-1],
                frame_id=self.BASE_LINK,
            )

            future = self._moveit2._plan_cartesian_path(max_step=step)
            if future is None:
                self.get_logger().error("_plan_cartesian_path devolvió None.")
                return None, False
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            # plan = self._moveit2.get_trajectory(future)

            result = future.result()
            if result is None or result.fraction < 0.5:
                self.get_logger().error(
                    f"Path cartesiano incompleto (fraction={result.fraction if result else 0:.2f})."
                )
                return None, False
            plan = result.solution

            if plan is None:
                self.get_logger().error("No se obtuvo trayectoria cartesiana.")
                return None, False

            all_plans.append(plan)

            if joint_names is None:
                joint_names = list(plan.joint_trajectory.joint_names)

            # Avanzar start state al último punto del plan
            last_pt = plan.joint_trajectory.points[-1]
            self._moveit2.set_joint_goal(
                joint_positions=list(last_pt.positions),
                joint_names=joint_names,
            )

        # Resetear start state al estado actual real
        self._moveit2.clear_goal_constraints()

        # ---- FK para cada punto de todos los planes ----
        # Usamos /compute_fk (servicio estándar de MoveIt2)
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
                    self.get_logger().error("FK falló en un punto de la trayectoria.")
                    return None, False

                # Convertir a mm (igual que el original)
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

        # ---- Límites de velocidad articular desde URDF (igual que ROS1) ----
        try:
            robot_desc = (
                self.get_parameter("/lbr/robot_description")
                .get_parameter_value()
                .string_value
            )
        except Exception:
            robot_desc = ""
            self.get_logger().warn(
                "Parámetro robot_description no encontrado; "
                "no se aplicarán límites articulares."
            )

        vel_limit = {}
        if robot_desc:
            root = ET.fromstring(robot_desc)
            for child in root:
                if child.tag == "joint" and child.get("type") == "revolute":
                    j_name = child.get("name")
                    for attrib in child:
                        if attrib.tag == "limit":
                            vel_limit[j_name] = float(attrib.get("velocity")) * 0.9

        # ---- Recalcular tiempos con perfiles de velocidad ----
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

        # ---- Merge lineal + angular (tomar el dt mayor) ----
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

        # ---- Enforcement de límites articulares ----
        n_joints = len(all_plans[0].joint_trajectory.points[0].positions)
        zero_Jvel = [0.0] * n_joints
        full_corrected_traj_with_limits = copy.deepcopy(full_corrected_traj)

        for i in range(len(full_corrected_traj) - 1):
            time_diff = (
                full_corrected_traj[i + 1]["time"] - full_corrected_traj[i]["time"]
            )
            updated_new_times = []
            update_time = (
                False  # FIX: resetear por punto, no arrastrar entre iteraciones
            )

            for j in range(n_joints):
                angle_diff = 0.0
                new_Jaccel = 0.0
                new_Jspeed = 0.0

                if time_diff != 0:
                    angle_diff = (
                        full_corrected_traj[i + 1]["state"][j]
                        - full_corrected_traj[i]["state"][j]
                    )
                    new_Jaccel = (
                        angle_diff
                        - full_corrected_traj_with_limits[i]["Jspeed"][j] * time_diff
                    ) * (2 / time_diff**2)
                    new_Jspeed = (
                        full_corrected_traj_with_limits[i]["Jspeed"][j]
                        + new_Jaccel * time_diff
                    )

                # Lookup del límite articular usando joint_names (no rs.joint_state.name)
                j_name = (
                    joint_names[j] if (joint_names and j < len(joint_names)) else ""
                )
                limit = vel_limit.get(j_name, float("inf"))

                if limit < new_Jspeed or time_diff == 0:
                    new_Jspeed = limit
                    if angle_diff != 0:
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
                            new_Jspeed - full_corrected_traj_with_limits[i]["Jspeed"][j]
                        )
                        / new_Jaccel
                    )
                    updated_new_times.append(new_time_step)
                    update_time = True

                full_corrected_traj_with_limits[i + 1]["Jaccel"][j] = new_Jaccel
                full_corrected_traj_with_limits[i + 1]["Jspeed"][j] = new_Jspeed

            full_corrected_traj_with_limits[i + 1]["time"] = (
                full_corrected_traj_with_limits[i]["time"] + time_diff
            )

            if update_time:
                self.get_logger().warn(
                    "Límite de velocidad articular excedido — reescalando."
                )
                time_diff = max(updated_new_times)
                if time_diff == 0: # problemas para computar los divisibles por frames
                    continue  # nada que reescalar
                full_corrected_traj_with_limits[i + 1]["time"] = (
                    full_corrected_traj_with_limits[i]["time"] + time_diff
                )
                for j in range(n_joints):
                    angle_diff = (
                        full_corrected_traj[i + 1]["state"][j]
                        - full_corrected_traj[i]["state"][j]
                    )
                    new_Jaccel = (
                        angle_diff
                        - full_corrected_traj_with_limits[i]["Jspeed"][j] * time_diff
                    ) * (2 / time_diff**2)
                    new_Jspeed = (
                        full_corrected_traj_with_limits[i]["Jspeed"][j]
                        + new_Jaccel * time_diff
                    )
                    full_corrected_traj_with_limits[i + 1]["Jaccel"][j] = new_Jaccel
                    full_corrected_traj_with_limits[i + 1]["Jspeed"][j] = new_Jspeed

            if extra_info and t_accel == full_corrected_traj[i + 1]["time"]:
                t_accel = full_corrected_traj_with_limits[i + 1]["time"]
            if extra_info and t_dec == full_corrected_traj[i + 1]["time"]:
                t_dec = full_corrected_traj_with_limits[i + 1]["time"]

        full_corrected_traj_with_limits[-1]["Jspeed"] = copy.deepcopy(zero_Jvel)

        # ---- Construir el mensaje RobotTrajectory ----
        new_plan = RobotTrajectory()
        new_plan.joint_trajectory.header.frame_id = self.BASE_LINK
        new_plan.joint_trajectory.joint_names = list(joint_names) if joint_names else []

        for state in full_corrected_traj_with_limits:
            point = JointTrajectoryPoint()
            point.positions = copy.deepcopy(state["state"])
            point.velocities = copy.deepcopy(state["Jspeed"])
            point.accelerations = copy.deepcopy(state["Jaccel"])
            point.effort = []
            t_secs = int(state["time"])
            t_nsecs = int((state["time"] - t_secs) * 1_000_000_000)
            point.time_from_start = Duration(sec=t_secs, nanosec=t_nsecs)
            new_plan.joint_trajectory.points.append(point)

        if extra_info:
            return new_plan, success, t_accel, t_dec
        return new_plan, success


def main(args=None):
    rclpy.init(args=args)
    control = MoveGroupPythonIntefaceControl()

    try:
        offset = 0.1

        # Esponja verde
        R = -math.pi
        P = -4.15 * math.pi / 180.0
        Y = math.pi / 2.0
        z0 = 0.3651

        x0 = 0.0
        y0 = 0.543

        quat = quaternion_from_euler(R, P, Y)
        q_norm = math.sqrt(sum(q**2 for q in quat))

        target = Pose()
        target.orientation.x = quat[0] / q_norm
        target.orientation.y = quat[1] / q_norm
        target.orientation.z = quat[2] / q_norm
        target.orientation.w = quat[3] / q_norm

        # Mover a pose de aproximación
        target.position.x = x0
        target.position.y = y0
        target.position.z = z0 + offset
        control.go_to_pose(target)

        control.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))

        penetrations = [0.001, 0.003, 0.005, 0.007]  # m
        speeds = [70.0, 50.0, 30.0, 10.0]  # mm/s

        # Test de histéresis
        for p in penetrations:
            control.get_logger().info(f"Penetración: {p}")

            for s in speeds:
                control.get_logger().info(f"Velocidad: {s}")

                # Reset sensor FT (descomentar cuando esté portado):
                # reset(control, True)

                target.position.x = x0
                target.position.y = y0
                target.position.z = z0 - p
                control.go_to_pose_speed(target, s, accel=2000.0)

                # Tiempo de penetración
                control.get_clock().sleep_for(rclpy.duration.Duration(seconds=100))

                target.position.x = x0
                target.position.y = y0
                target.position.z = z0 + offset
                control.go_to_pose_speed(target, 100.0, accel=100.0)

                # Tiempo de descanso
                control.get_clock().sleep_for(rclpy.duration.Duration(seconds=400))

    except KeyboardInterrupt:
        pass
    finally:
        control.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
