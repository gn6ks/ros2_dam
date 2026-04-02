import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from lbr_bringup.moveit import LBRMoveGroupMixin


def generate_launch_description():

    # ── Argumentos ──────────────────────────────────────────────────────────
    robot_name_arg = DeclareLaunchArgument(
        name="robot_name",
        default_value="lbr",
    )
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value="iiwa7",
        description="Modelo del robot: iiwa7, iiwa14, med7, med14",
    )

    # ── Configuración MoveIt ─────────────────────────────────────────────────
    moveit_configs = (
        LBRMoveGroupMixin.moveit_configs_builder(
            robot_name="iiwa7",
            package_name="iiwa7_moveit_config",
        )
        .to_moveit_configs()
    )

    # ── move_group node ──────────────────────────────────────────────────────
    move_group_params = LBRMoveGroupMixin.params_move_group()

    move_group_node = LBRMoveGroupMixin.node_move_group(
        parameters=[
            moveit_configs.to_dict(),
            move_group_params,
            {"use_sim_time": True},
        ],
        namespace=LaunchConfiguration("robot_name"),
    )

    # Delay para que Gazebo esté listo antes de arrancar move_group
    move_group_node_delayed = TimerAction(
        period=5.0,
        actions=[move_group_node],
    )

    # ── RViz2 ────────────────────────────────────────────────────────────────
    rviz_config = os.path.join(
        get_package_share_directory("iiwa7_moveit_config"),
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_configs.planning_pipelines,
            moveit_configs.robot_description_kinematics,
            {"use_sim_time": True},
        ],
        # ← remappings corregidos para el namespace /lbr
        remappings=[
            ("/monitored_planning_scene",       "/lbr/monitored_planning_scene"),
            ("/planning_scene",                 "/lbr/planning_scene"),
            ("/display_planned_path",           "/lbr/display_planned_path"),
            ("/robot_description",              "/lbr/robot_description"),
            ("/robot_description_semantic",     "/lbr/robot_description_semantic"),
            ("/joint_states",                   "/lbr/joint_states"),
            ("/move_group/display_planned_path","/lbr/move_group/display_planned_path"),
        ],
        output="screen",
    )

    # Delay para RViz — arranca después del move_group
    rviz_node_delayed = TimerAction(
        period=10.0,
        actions=[rviz_node],
    )

    # ── Tu nodo ──────────────────────────────────────────────────────────────
    your_node = Node(
        package="press_moveit_ft45_pkg",   # ← quitado el espacio extra que tenías
        executable="press_motion_node",
        name="press_motion_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
        remappings=[
            ("/joint_states", "/lbr/joint_states"),
        ],
    )

    your_node_delayed = TimerAction(
        period=15.0,   # arranca al final, cuando todo está listo
        actions=[your_node],
    )

    # ── Launch description ───────────────────────────────────────────────────
    return LaunchDescription([
        LBRMoveGroupMixin.arg_allow_trajectory_execution(),
        LBRMoveGroupMixin.args_publish_monitored_planning_scene(),
        LBRMoveGroupMixin.arg_capabilities(),
        LBRMoveGroupMixin.arg_disable_capabilities(),
        robot_name_arg,
        model_arg,
        move_group_node_delayed,   # 5s delay
        rviz_node_delayed,         # 10s delay
        your_node_delayed,         # 15s delay
    ])