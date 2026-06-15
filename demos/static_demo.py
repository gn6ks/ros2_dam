#! /usr/bin/env python3

import sys
import os
import copy
import time
import math
import numpy as np
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Install: https://moveit.picknik.ai/main/doc/examples/moveit_py/moveitpy_simple_example.html
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters
from moveit.core.robot_state import RobotState
from moveit.core.kinematic_constraints import construct_joint_constraint

import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from std_msgs.msg import Header, Float64MultiArray, MultiArrayDimension
from sensor_msgs.msg import JointState
from moveit_msgs.msg import (
    RobotTrajectory,
    DisplayTrajectory,
    MoveItErrorCodes,
)
from moveit_msgs.msg import RobotState as MoveItRobotState
from trajectory_msgs.msg import JointTrajectoryPoint
import PyKDL

# tf_transformations (ros2 package: tf_transformations)
# pip install transforms3d   OR   apt install ros-<distro>-tf-transformations
from tf_transformations import quaternion_from_euler

# Custom service stubs — adapt to your actual ROS2 service definitions
# from iiwa_tools.srv import GetFK          # --> keep if ported to ROS2
# from ft17_publisher.srv import Reset      # --> keep if ported to ROS2

def all_close(goal, actual, tolerance):
    """
    Test whether a list of values (or Pose / PoseStamped) are within tolerance.
    """
    if isinstance(goal, list):
        for i in range(len(goal)):
            if abs(actual[i] - goal[i]) > tolerance:
                return False
        return True
        
    elif isinstance(goal, PoseStamped):
        return all_close(goal.pose, actual.pose, tolerance)
        
    elif isinstance(goal, Pose):
        def pose_to_list(p):
            return [
                p.position.x, p.position.y, p.position.z,
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w,
            ]
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
    return True

class EEF:
    """Describes an end-effector tool."""

    def __init__(
        self,
        EE_end_frame: PyKDL.Frame = PyKDL.Frame(),
        x: float = 0,
        y: float = 0,
        z: float = 0,
        ATC_frame: PyKDL.Frame = PyKDL.Frame(),
        name: str = "",
        path: str = "",
    ):
        self.EE_end_frame = EE_end_frame
        self.x = x
        self.y = y
        self.z = z
        self.ATC_frame = ATC_frame
        self.name = name
        self.path = path


class MoveGroupPythonIntefaceControl(Node):
    """
    ROS2 port of MoveGroupPythonIntefaceControl.

    Inherits from rclpy.node.Node so that all ROS2 communication
    (publishers, service clients, parameters) is encapsulated here.
    """

    def __init__(self):
        super().__init__("move_group_control", namespace="/lbr")
        # self._moveit = MoveItPy(node_name="move_group_control", name_space="/lbr")

        from moveit_configs_utils import MoveItConfigsBuilder
        from ament_index_python.packages import get_package_share_directory
        
        moveit_config = (
            MoveItConfigsBuilder("iiwa7", package_name="lbr_moveit_config")
            .to_moveit_configs()
        )
        self._moveit = MoveItPy(config_dict=moveit_config)
        self._robot = self._moveit.get_robot_model()          # RobotModel
        self._planning_scene = self._moveit.get_planning_scene_monitor()

        group_name = "manipulator"
        self._arm = self._moveit.get_planning_component(group_name)  # PlanningComponent

        # End-effector link name (read from planning component)
        self.eef_link = self._arm.get_end_effector_link()
        self.get_logger().info(f"End effector link: {self.eef_link}")

        # Publisher for trajectory visualisation in RViz2
        self._display_traj_pub = self.create_publisher(
            DisplayTrajectory,
            "display_planned_path",
            20,
        )

        # Service clients -------------------------------------------------------
        # FT sensor reset  (adapt srv type to your ROS2 port of ft17_publisher)
        # self._ft_reset_client = self.create_client(Reset, "FT_reset")

        # Forward kinematics  (adapt srv type to your ROS2 port of iiwa_tools)
        # from iiwa_tools.srv import GetFK
        # self._fk_client = self.create_client(GetFK, "/iiwa_fk_server")
        self.box_name = ""
        self.get_logger().info("MoveGroupPythonIntefaceControl (ROS2) ready.")

    def reset(self, req: bool) -> bool:
        """
        Call the FT sensor reset service.
        Equivalent to the standalone reset() function in the ROS1 version.
        """
        self.get_logger().info("Calibrating FT sensor …")
        # ROS2 async service call pattern:
        # if not self._ft_reset_client.wait_for_service(timeout_sec=5.0):
        #     self.get_logger().error("FT_reset service not available")
        #     return False
        # future = self._ft_reset_client.call_async(Reset.Request())
        # rclpy.spin_until_future_complete(self, future)
        # return future.result().res
        raise NotImplementedError("Adapt Reset service type from your ROS2 ft17_publisher port")

    def go_to_joint_state(self) -> bool:
        """Move to a hard-coded joint configuration (same values as ROS1)."""
        joint_goal = [
            90.0  * math.pi / 180.0,
             0.0  * math.pi / 180.0,
             0.0  * math.pi / 180.0,
            -90.0 * math.pi / 180.0,
             0.0  * math.pi / 180.0,
             90.0 * math.pi / 180.0,
             0.0,
        ]

        self._arm.set_start_state_to_current_state()

        # Build a joint constraint goal
        with self._planning_scene.read_only() as scene:
            robot_state = scene.current_state
            robot_state.set_joint_group_positions(
                "manipulator", joint_goal
            )
            robot_state.update()

        self._arm.set_goal_state(robot_state=robot_state)
        plan_result = self._arm.plan()
        if plan_result:
            self._moveit.execute(plan_result.trajectory, controllers=[])
        else:
            self.get_logger().error("go_to_joint_state: planning failed")
            return False

        # Verify
        with self._planning_scene.read_only() as scene:
            current = list(
                scene.current_state.get_joint_group_positions("manipulator")
            )
        return all_close(joint_goal, current, 0.01)

    def go_to_pose(self, pose: Pose) -> bool:
        """Plan and execute motion to a Cartesian pose (no speed control)."""
        self._arm.set_start_state_to_current_state()

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "lbr/link_0"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose

        self._arm.set_goal_state(
            pose_stamped_msg=pose_stamped,
            pose_link=self.eef_link,
        )

        plan_result = self._arm.plan()
        if plan_result:
            self._moveit.execute(plan_result.trajectory, controllers=[])
        else:
            self.get_logger().error("go_to_pose: planning failed")
            return False

        with self._planning_scene.read_only() as scene:
            current_pose = scene.current_state.get_pose(self.eef_link)
        return all_close(pose, current_pose, 0.01)


    def follow_trajectory_speed(
        self,
        waypoints: list,
        speeds: list,
        ang_speeds: list = [],
        accel: float = 100.0,
    ):
        """Execute a multi-segment Cartesian trajectory with speed control."""
        plan, success = self.compute_cartesian_path_velocity_control(
            waypoints, speeds, EE_ang_speed=ang_speeds, max_linear_accel=accel
        )
        self._publish_and_execute(plan, success)
        time.sleep(0.5)

    def go_to_pose_speed(
        self,
        pose: Pose,
        speed: float = 10.0,
        ang_speed: list = [],
        accel: float = 100.0,
    ):
        """Move to a Cartesian pose at a controlled EEF speed (mm/s)."""
        with self._planning_scene.read_only() as scene:
            current_pose = scene.current_state.get_pose(self.eef_link)

        waypoints = [
            [copy.deepcopy(current_pose), copy.deepcopy(pose)]
        ]
        plan, success = self.compute_cartesian_path_velocity_control(
            waypoints, [speed], EE_ang_speed=ang_speed, max_linear_accel=accel
        )
        self._publish_and_execute(plan, success)
        time.sleep(0.5)

    def _publish_and_execute(self, plan, success: bool):
        """Publish trajectory to RViz2 and execute if planning succeeded."""
        if plan:
            display_trajectory = DisplayTrajectory()
            display_trajectory.trajectory.append(plan)
            self._display_traj_pub.publish(display_trajectory)
            self.get_logger().info("Plan published to RViz2.")
        else:
            self.get_logger().error("Failed to compute or publish plan.")

        if success and plan:
            self._moveit.execute(plan, controllers=[])

    def wait_for_state_update(
        self,
        box_is_known: bool = False,
        box_is_attached: bool = False,
        timeout: float = 4.0,
    ) -> bool:
        """Poll the planning scene until the collision object state matches expectations."""
        start = time.time()
        while time.time() - start < timeout:
            with self._planning_scene.read_only() as scene:
                attached = (
                    self.box_name
                    in scene.get_attached_objects([self.box_name])
                )
                known = self.box_name in scene.get_known_object_names()
            if (box_is_attached == attached) and (box_is_known == known):
                return True
            time.sleep(0.1)
        return False

    def add_box(self, timeout: float = 4.0) -> bool:
        box_pose = PoseStamped()
        box_pose.header.frame_id = "panda_leftfinger"  # adapt frame as needed
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.07
        self.box_name = "box"
        with self._planning_scene.read_write() as scene:
            scene.add_box(self.box_name, box_pose, size=(0.1, 0.1, 0.1))
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout: float = 4.0) -> bool:
        with self._planning_scene.read_write() as scene:
            scene.attach_box(self.eef_link, self.box_name, touch_links=[])
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, timeout: float = 4.0) -> bool:
        with self._planning_scene.read_write() as scene:
            scene.remove_attached_object(self.eef_link, name=self.box_name)
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_box(self, timeout: float = 4.0) -> bool:
        with self._planning_scene.read_write() as scene:
            scene.remove_world_object(self.box_name)
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )

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
        frame.p = PyKDL.Vector(
            pose.position.x, pose.position.y, pose.position.z
        )
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
        M = frame.M
        p = frame.p
        x = -(p[0]*M[0,0] + p[1]*M[1,0] + p[2]*M[2,0])
        y = -(p[0]*M[0,1] + p[1]*M[1,1] + p[2]*M[2,1])
        z = -(p[0]*M[0,2] + p[1]*M[1,2] + p[2]*M[2,2])
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
        f12 = f1.Inverse() * f2
        return abs(f12.M.GetRotAngle()[0])

    def compute_lin_or_ang_distance(
        self, pose1: Pose, pose2: Pose, linear: bool = True
    ) -> float:
        if linear:
            return self.compute_distance(pose1, pose2)
        return self.compute_angle_distance(pose1, pose2)

    def get_shifted_pose(self, origin_pose: Pose, shift: list) -> Pose:
        """
        shift = [dx, dy, dz, rx, ry, rz]  (metres and radians)
        """
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
        self,
        initial_pose: Pose,
        final_pose: Pose,
        step_pos_min: float,
        step_deg_min: float,
        n_points_max: int,
    ):
        """Identical interpolation logic to ROS1."""
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
                    * float(point) / float(n_points)
                )
                y = initial_pose.position.y + (
                    (final_pose.position.y - initial_pose.position.y)
                    * float(point) / float(n_points)
                )
                z = initial_pose.position.z + (
                    (final_pose.position.z - initial_pose.position.z)
                    * float(point) / float(n_points)
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
        """
        Recalculate trajectory times to honour target EEF speeds.
        Only logging calls where changed from last refactor
        """
        thres = 0.05
        if not linear:
            thres *= 0.7 * (math.pi / 180)

        corrected_traj = []
        n_joints = len(all_plans[0].joint_trajectory.points[0].positions)
        zero_Jvel = [0.0] * n_joints

        corrected_traj.append({
            "pose":     traj_poses[0][0],
            "state":    all_plans[0].joint_trajectory.points[0].positions,
            "EE_speed": 0,
            "EE_accel": 0,
            "time":     0,
            "Jspeed":   copy.deepcopy(zero_Jvel),
            "Jaccel":   copy.deepcopy(zero_Jvel),
        })

        success = True
        i = 0
        s_i = 1
        v_chg_i = 0
        init_speed_change = False
        final_speed_change = False

        for plan_poses in traj_poses:
            if traj_mov[i] < thres:
                j = -1
                for pose in plan_poses:
                    j += 1
                    if pose == corrected_traj[-1]["pose"]:
                        continue
                    corrected_traj.append({
                        "pose":     pose,
                        "state":    all_plans[i].joint_trajectory.points[j].positions,
                        "EE_speed": 0,
                        "EE_accel": 0,
                        "time":     corrected_traj[-1]["time"],
                        "Jspeed":   copy.deepcopy(zero_Jvel),
                        "Jaccel":   copy.deepcopy(zero_Jvel),
                    })
                continue  # NOTE: i is NOT incremented here intentionally

            init_speed_change  = EE_speed_aux[s_i] > EE_speed_aux[s_i - 1]
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
                    ) < thres
                    and not init_speed_change
                ):
                    corrected_traj.append({
                        "pose":     corrected_traj[-1]["pose"],
                        "state":    all_plans[i].joint_trajectory.points[j].positions,
                        "EE_speed": 0,
                        "EE_accel": 0,
                        "time":     corrected_traj[-1]["time"],
                        "Jspeed":   copy.deepcopy(zero_Jvel),
                        "Jaccel":   copy.deepcopy(zero_Jvel),
                    })
                    continue

                # --- Acceleration at the beginning of the speed section ---
                if init_speed_change:
                    if final_speed_change:
                        if (
                            v_change[v_chg_i]["x_min_req"]
                            + v_change[v_chg_i + 1]["x_min_req"]
                        ) > traj_mov[i]:
                            t2 = (
                                -2 * EE_speed_aux[s_i + 1]
                                + math.sqrt(
                                    2 * (
                                        EE_speed_aux[s_i + 1] ** 2
                                        + EE_speed_aux[s_i - 1] ** 2
                                        + 2 * max_accel * traj_mov[i]
                                    )
                                )
                            ) / (2 * max_accel)
                            v_change[v_chg_i + 1]["x_min_req"] = (
                                (EE_speed_aux[s_i + 1] + max_accel * t2) * t2
                                - 0.5 * max_accel * t2 ** 2
                            )
                            v_change[v_chg_i]["x_min_req"] = (
                                traj_mov[i] - v_change[v_chg_i + 1]["x_min_req"]
                            )

                    x_plan += self.compute_lin_or_ang_distance(
                        pose, corrected_traj[-1]["pose"], linear
                    )
                    corrected_traj.append({
                        "pose":     pose,
                        "state":    all_plans[i].joint_trajectory.points[j].positions,
                        "EE_speed": 0,
                        "EE_accel": 0,
                        "time":     0,
                        "Jspeed":   copy.deepcopy(zero_Jvel),
                        "Jaccel":   copy.deepcopy(zero_Jvel),
                    })
                    init_transition_indexes.append(len(corrected_traj) - 1)

                    next_dist = self.compute_lin_or_ang_distance(
                        pose,
                        plan_poses[min(j + 1, len(plan_poses) - 1)],
                        linear,
                    )
                    if (x_plan + next_dist) > v_change[v_chg_i]["x_min_req"] or (
                        traj_mov[i] < v_change[v_chg_i]["x_min_req"]
                        and j >= (len(plan_poses) - 1)
                    ):
                        speed_diff = EE_speed_aux[s_i] - EE_speed_aux[s_i - 1]
                        trans_accel = min(
                            (2 * speed_diff * EE_speed_aux[s_i - 1] + speed_diff ** 2)
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
                            disc = tB ** 2 - 4 * tA * tC
                            t1 = (-tB + math.sqrt(disc)) / (2 * tA)
                            t2_ = (-tB - math.sqrt(disc)) / (2 * tA)
                            if t1 < 0 and t2_ < 0:
                                new_time = 0
                                success = False
                            elif t1 < 0:
                                new_time = t2_
                            elif t2_ < 0:
                                new_time = t1
                            elif t1 < t2_:
                                new_time = t1
                            else:
                                new_time = t2_
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

                # --- Deceleration at the end of the speed section ---
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
                            speed_diff = EE_speed_aux[s_i + 1] - corrected_traj[-1]["EE_speed"]
                            ta1 = (
                                2 * speed_diff * EE_speed_aux[s_i] + speed_diff ** 2
                            ) / (2 * x_trans)
                            ta2 = -max_accel
                            trans_accel = ta2 if abs(ta1) > abs(ta2) else ta1
                            first_final = False
                            corrected_traj[-1]["EE_accel"] = trans_accel

                        if speed_diff == 0:
                            corrected_traj.append({
                                "pose":     pose,
                                "state":    all_plans[i].joint_trajectory.points[j].positions,
                                "EE_speed": 0,
                                "EE_accel": 0,
                                "time":     corrected_traj[-1]["time"],
                                "Jspeed":   copy.deepcopy(zero_Jvel),
                                "Jaccel":   copy.deepcopy(zero_Jvel),
                            })
                            continue

                        tA = trans_accel / 2
                        tB = corrected_traj[-1]["EE_speed"]
                        tC = -self.compute_lin_or_ang_distance(
                            pose, corrected_traj[-1]["pose"], linear
                        )
                        disc = tB ** 2 - 4 * tA * tC
                        if disc <= 0:
                            new_time = (-tB) / (2 * tA)
                        else:
                            t1 = (-tB + math.sqrt(disc)) / (2 * tA)
                            t2_ = (-tB - math.sqrt(disc)) / (2 * tA)
                            if t1 < 0 and t2_ < 0:
                                new_time = 0
                                success = False
                            elif t1 < 0:
                                new_time = t2_
                            elif t2_ < 0:
                                new_time = t1
                            elif t1 < t2_:
                                new_time = t1
                            else:
                                new_time = t2_

                        new_total_time = corrected_traj[-1]["time"] + new_time
                        new_speed = corrected_traj[-1]["EE_speed"] + trans_accel * new_time
                        corrected_traj.append({
                            "pose":     pose,
                            "state":    all_plans[i].joint_trajectory.points[j].positions,
                            "EE_speed": new_speed,
                            "EE_accel": trans_accel,
                            "time":     new_total_time,
                            "Jspeed":   copy.deepcopy(zero_Jvel),
                            "Jaccel":   copy.deepcopy(zero_Jvel),
                        })
                        if (linear and x_left < 0.1) or (
                            not linear and x_left < 0.07 * (math.pi / 180)
                        ):
                            corrected_traj[-1]["EE_accel"] = 0
                            v_chg_i += 1
                            final_speed_change = False
                    else:
                        corrected_traj.append({
                            "pose":     pose,
                            "state":    all_plans[i].joint_trajectory.points[j].positions,
                            "EE_speed": EE_speed_aux[s_i],
                            "EE_accel": 0,
                            "time":     corrected_traj[-1]["time"]
                            + self.compute_lin_or_ang_distance(
                                pose, corrected_traj[-1]["pose"], linear
                            ) / EE_speed_aux[s_i],
                            "Jspeed":   copy.deepcopy(zero_Jvel),
                            "Jaccel":   copy.deepcopy(zero_Jvel),
                        })

                # --- Constant speed ---
                else:
                    corrected_traj.append({
                        "pose":     pose,
                        "state":    all_plans[i].joint_trajectory.points[j].positions,
                        "EE_speed": EE_speed_aux[s_i],
                        "EE_accel": 0,
                        "time":     corrected_traj[-1]["time"]
                        + self.compute_lin_or_ang_distance(
                            pose, corrected_traj[-1]["pose"], linear
                        ) / EE_speed_aux[s_i],
                        "Jspeed":   copy.deepcopy(zero_Jvel),
                        "Jaccel":   copy.deepcopy(zero_Jvel),
                    })

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
        """
        Generate a speed-controlled Cartesian trajectory (ROS2 port).

        Major ROS2 differences vs ROS1:
          - compute_cartesian_path   --> arm.compute_cartesian_path() (MoveItPy)
          - FK service               --> use MoveItPy's built-in FK or your ROS2 port of iiwa_fk_server
          - robot_description param  --> node.get_parameter() / rclpy approach
          - RobotState               --> moveit.core.robot_state.RobotState
        """
        success = True

        # --- Angular speed defaults
        if not EE_ang_speed:
            EE_ang_speed = [s * 0.7 for s in EE_speed]

        EE_ang_speed = [a * (math.pi / 180) for a in EE_ang_speed]
        max_ang_accel *= math.pi / 180

        # --- Build speed profile
        EE_speed_aux     = [0] + list(EE_speed)     + [0]
        EE_ang_speed_aux = [0] + list(EE_ang_speed) + [0]
        v_change     = []
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
            v_change.append({
                "t_req":     t_lin,
                "x_min_req": EE_speed_aux[i] * t_lin + (a_lin * t_lin ** 2) / 2,
            })
            v_change_ang.append({
                "t_req":     t_ang,
                "x_min_req": EE_ang_speed_aux[i] * t_ang + (a_ang * t_ang ** 2) / 2,
            })

        # --- Cartesian path planning (MoveItPy)
        # NOTE: MoveItPy's compute_cartesian_path API may differ slightly from
        # moveit_commander.  Adapt the call signature to your MoveItPy version.
        all_plans = []
        for traj in waypoints_list:
            self._arm.set_start_state_to_current_state()
            plan_result = self._arm.compute_cartesian_path(
                waypoints=traj,
                max_step=step,
                jump_threshold=0.0,
            )
            if plan_result is None:
                self.get_logger().error("compute_cartesian_path returned None")
                return None, False
            all_plans.append(plan_result.trajectory)

            # Update start state for the next segment
            last_pt = plan_result.trajectory.joint_trajectory.points[-1]
            with self._planning_scene.read_write() as scene:
                rs = scene.current_state
                rs.set_joint_group_positions(
                    "manipulator", list(last_pt.positions)
                )
                rs.update()
            self._arm.set_start_state(robot_state=rs)

        self._arm.set_start_state_to_current_state()

        # --- Forward kinematics to get EEF poses
        # Option A: use your ROS2-ported iiwa_fk_server service (same interface)
        # Option B: use MoveItPy built-in FK (shown below as pseudocode)
        #
        # Here we use the MoveItPy built-in FK as the primary path.
        # If you prefer the service, restore the service client pattern from ROS1
        # and adapt accordingly.

        joint_names = all_plans[0].joint_trajectory.joint_names
        traj_poses = []
        traj_mov_position = []
        traj_mov_angle    = []

        for plan in all_plans:
            plan_poses = []
            traj_mov_i_pos = 0.0
            traj_mov_i_ang = 0.0

            with self._planning_scene.read_only() as scene:
                rs = scene.current_state

            for joint_state_pt in plan.joint_trajectory.points:
                rs.set_joint_group_positions(
                    "manipulator", list(joint_state_pt.positions)
                )
                rs.update()
                pose_m = rs.get_pose(self.eef_link)       # returns geometry_msgs/Pose

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

        # --- Joint velocity limits from URDF ---
        # In ROS2 the robot_description is a node parameter
        try:
            robot_desc = self.get_parameter("robot_description").get_parameter_value().string_value
        except Exception:
            robot_desc = ""
            self.get_logger().warn("robot_description parameter not found; joint limits not applied.")

        vel_limit = {}
        if robot_desc:
            root = ET.fromstring(robot_desc)
            for child in root:
                if child.tag == "joint" and child.get("type") == "revolute":
                    j_name = child.get("name")
                    for attrib in child:
                        if attrib.tag == "limit":
                            vel_limit[j_name] = float(attrib.get("velocity")) * 0.9

        # --- Adjust times
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

        # --- Merge linear and angular time profiles
        full_corrected_traj = copy.deepcopy(corrected_traj)
        for i in range(len(corrected_traj) - 1):
            dt_lin = corrected_traj[i + 1]["time"]     - corrected_traj[i]["time"]
            dt_ang = corrected_traj_ang[i + 1]["time"] - corrected_traj_ang[i]["time"]
            full_corrected_traj[i + 1]["time"] = (
                full_corrected_traj[i]["time"] + max(dt_lin, dt_ang)
            )

        # --- Joint velocity limit enforcement
        n_joints = len(all_plans[0].joint_trajectory.points[0].positions)
        zero_Jvel = [0.0] * n_joints
        full_corrected_traj_with_limits = copy.deepcopy(full_corrected_traj)

        for i in range(len(full_corrected_traj) - 1):
            time_diff = (
                full_corrected_traj[i + 1]["time"] - full_corrected_traj[i]["time"]
            )
            updated_new_times = []
            update_time = False

            for j in range(len(full_corrected_traj[i]["state"])):
                if time_diff != 0:
                    angle_diff = (
                        full_corrected_traj[i + 1]["state"][j]
                        - full_corrected_traj[i]["state"][j]
                    )
                    new_Jaccel = (
                        angle_diff
                        - full_corrected_traj_with_limits[i]["Jspeed"][j] * time_diff
                    ) * (2 / time_diff ** 2)
                    new_Jspeed = (
                        full_corrected_traj_with_limits[i]["Jspeed"][j]
                        + new_Jaccel * time_diff
                    )

                j_name = joint_names[j] if j < len(joint_names) else ""
                limit = vel_limit.get(j_name, float("inf"))
                if limit < new_Jspeed or time_diff == 0:
                    new_Jspeed = limit
                    new_Jaccel = (
                        2 * full_corrected_traj_with_limits[i]["Jspeed"][j]
                        * (new_Jspeed - full_corrected_traj_with_limits[i]["Jspeed"][j])
                        + (new_Jspeed - full_corrected_traj_with_limits[i]["Jspeed"][j]) ** 2
                    ) / (2 * angle_diff) if angle_diff != 0 else 0.0
                    new_time_step = (
                        (angle_diff / new_Jspeed)
                        if abs(new_Jaccel) < 0.0001
                        else (new_Jspeed - full_corrected_traj_with_limits[i]["Jspeed"][j])
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
                self.get_logger().warn("Joint speed limit exceeded — rescaling timestep.")
                time_diff = max(updated_new_times)
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
                    ) * (2 / time_diff ** 2)
                    new_Jspeed = (
                        full_corrected_traj_with_limits[i]["Jspeed"][j]
                        + new_Jaccel * time_diff
                    )
                    full_corrected_traj_with_limits[i + 1]["Jaccel"][j] = new_Jaccel
                    full_corrected_traj_with_limits[i + 1]["Jspeed"][j] = new_Jspeed

        full_corrected_traj_with_limits[-1]["Jspeed"] = copy.deepcopy(zero_Jvel)

        # --- Build RobotTrajectory msg
        new_plan = RobotTrajectory()
        new_plan.joint_trajectory.header.frame_id = "base_link"
        new_plan.joint_trajectory.joint_names = list(joint_names)

        for state in full_corrected_traj_with_limits:
            point = JointTrajectoryPoint()
            point.positions     = copy.deepcopy(state["state"])
            point.velocities    = copy.deepcopy(state["Jspeed"])
            point.accelerations = copy.deepcopy(state["Jaccel"])
            point.effort        = []
            t_secs  = int(state["time"])
            t_nsecs = int((state["time"] - t_secs) * 1_000_000_000)
            # ROS2 uses builtin_interfaces/Duration
            from builtin_interfaces.msg import Duration
            point.time_from_start = Duration(sec=t_secs, nanosec=t_nsecs)
            new_plan.joint_trajectory.points.append(point)

        if extra_info:
            # t_accel / t_dec tracking omitted for brevity
            return new_plan, success, 0.0, 0.0
        return new_plan, success

def main(args=None):
    rclpy.init(args=args)
    control = MoveGroupPythonIntefaceControl()

    try:
        # calcs for green spongebob
        R  = -math.pi
        P  = -4.15 * math.pi / 180.0
        Y  =  math.pi / 2.0
        z0 =  0.3651
        x0 =  0.0
        y0 =  0.543
        offset = 0.1

        # Build quaternion values
        quat = quaternion_from_euler(R, P, Y)
        q_norm = math.sqrt(sum(q ** 2 for q in quat))

        target = Pose()
        target.orientation.x = quat[0] / q_norm
        target.orientation.y = quat[1] / q_norm
        target.orientation.z = quat[2] / q_norm
        target.orientation.w = quat[3] / q_norm

        # Move to approach pose
        target.position.x = x0
        target.position.y = y0
        target.position.z = z0 + offset
        control.go_to_pose(target)

        # ROS2 sleep via Clock
        control.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))

        penetrations = [0.001, 0.003, 0.005, 0.007]  # m
        speeds       = [70.0,  50.0,  30.0,  10.0]   # mm/s

        # Hysteresis test
        for p in penetrations:
            control.get_logger().info(f"Penetration: {p}")
            for s in speeds:
                control.get_logger().info(f"Speed: {s}")

                # Reset FT sensor
                # control.reset(True)

                target.position.x = x0
                target.position.y = y0
                target.position.z = z0 - p
                control.go_to_pose_speed(target, s, accel=2000.0)

                control.get_clock().sleep_for(rclpy.duration.Duration(seconds=100))

                target.position.x = x0
                target.position.y = y0
                target.position.z = z0 + offset
                control.go_to_pose_speed(target, 100.0, accel=100.0)

                control.get_clock().sleep_for(rclpy.duration.Duration(seconds=400))

    except KeyboardInterrupt:
        pass
    finally:
        control.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()