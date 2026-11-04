import os
import sys

from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    LogInfo,
    EmitEvent,
)

from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ============================================================
    # 1. Configurações
    # ============================================================

    use_sim_time = LaunchConfiguration("use_sim_time")

    bringup_share = get_package_share_directory(
        "tcc_slam_bringup"
    )

    monitor_file = os.path.join(
        bringup_share,
        "launch",
        "readiness_monitor.py",
    )

    nav2_params = PathJoinSubstitution([
        FindPackageShare("tcc_slam_bringup"),
        "config",
        "nav2_params.yaml",
    ])

    # ============================================================
    # 2. Simulação
    # ============================================================

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("tcc_slam_bringup"),
                "launch",
                "simulation.launch.py",
            ])
        )
    )

    # ============================================================
    # 3. Monitor pré-Nav2
    # ============================================================

    pre_nav_monitor = ExecuteProcess(
        cmd=[
            sys.executable,
            monitor_file,
            "--mode",
            "pre_nav",
        ],
        output="screen",
    )

    # ============================================================
    # 4. Servidores Nav2
    # ============================================================

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "navigation_launch.py",
            ])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params,
            "autostart": "False",
        }.items(),
    )

    # ============================================================
    # 5. Lifecycle Manager
    # ============================================================

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": [
                "controller_server",
                "smoother_server",
                "planner_server",
                "route_server",
                "behavior_server",
                "velocity_smoother",
                "collision_monitor",
                "bt_navigator",
                "waypoint_follower",
            ],
        }],
    )

    # ============================================================
    # 6. Iniciar Nav2 após confirmação de prontidão
    # ============================================================

    def apos_monitor_pre_nav(event, context):

        if event.returncode != 0:

            return [
                EmitEvent(
                    event=Shutdown(
                        reason="Monitor pré-Nav2 terminou com erro."
                    )
                )
            ]

        return [
            LogInfo(
                msg="Mapa e TF disponíveis. Iniciando Nav2..."
            ),
            navigation_launch,
            lifecycle_manager,
        ]

    start_nav2_after_monitor = RegisterEventHandler(
        OnProcessExit(
            target_action=pre_nav_monitor,
            on_exit=apos_monitor_pre_nav,
        )
    )

    # ============================================================
    # 7. Launch Description
    # ============================================================

    return LaunchDescription([

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Utilizar relógio da simulação",
        ),

        start_nav2_after_monitor,

        simulation_launch,

        pre_nav_monitor,

    ])
