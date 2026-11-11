import os
import sys

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    RegisterEventHandler,
    IncludeLaunchDescription,
    LogInfo,
    EmitEvent,
)

from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ============================================================
    # 1. Diretórios
    # ============================================================

    description_share = get_package_share_directory(
        "tcc_slam_description"
    )

    bringup_share = get_package_share_directory(
        "tcc_slam_bringup"
    )

    slam_share = get_package_share_directory(
        "slam_toolbox"
    )

    xacro_file = os.path.join(
        description_share,
        "urdf",
        "tcc_robot.urdf.xacro",
    )

    world_file = os.path.join(
        description_share,
        "worlds",
        "tcc_world.sdf",
    )

    slam_config = os.path.join(
        bringup_share,
        "config",
        "slam_toolbox.yaml",
    )

    monitor_file = os.path.join(
        bringup_share,
        "launch",
        "readiness_monitor.py",
    )

    # ============================================================
    # 2. Gazebo
    # ============================================================

    gazebo = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-r",
            world_file,
        ],
        output="screen",
    )

    # ============================================================
    # 3. Gerar URDF
    # ============================================================

    generate_urdf = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "xacro",
            "xacro",
            xacro_file,
            "-o",
            "/tmp/tcc_robot.urdf",
        ],
        output="screen",
    )

    # ============================================================
    # 4. Robot State Publisher
    # ============================================================

    robot_description = ParameterValue(
        Command([
            "xacro",
            " ",
            xacro_file,
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
    )

    # ============================================================
    # 5. Bridge dos sensores
    # ============================================================

    sensor_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="sensor_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ],
    )

    # ============================================================
    # 6. Bridge TF
    # ============================================================

    tf_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="tf_bridge",
        output="screen",
        arguments=[
            "/model/tcc_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        remappings=[
            ("/model/tcc_robot/tf", "/tf"),
        ],
    )

    # ============================================================
    # 7. Ground Truth
    # ============================================================

    ground_truth_odom_node = Node(
        package="tcc_slam_ground_truth",
        executable="ground_truth_odom",
        name="ground_truth_odom",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"gazebo_topic": "/world/tcc_world/pose/info"},
            {"model_name": "tcc_robot"},
            {"world_frame": "world"},
            {"child_frame": "tcc_robot"},
        ],
    )

    # ============================================================
    # 8. Inserir robô
    # ============================================================

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot",
        output="screen",
        arguments=[
            "-world",
            "tcc_world",
            "-file",
            "/tmp/tcc_robot.urdf",
            "-name",
            "tcc_robot",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.10",
        ],
    )

    # ============================================================
    # 9. Monitor de prontidão pré-SLAM
    # ============================================================

    pre_slam_monitor = ExecuteProcess(
        cmd=[
            sys.executable,
            monitor_file,
            "--mode",
            "pre_slam",
        ],
        output="screen",
    )

    # ============================================================
    # 10. SLAM Toolbox
    # ============================================================

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                slam_share,
                "launch",
                "online_sync_launch.py",
            )
        ),
        launch_arguments={
            "slam_params_file": slam_config,
            "use_sim_time": "true",
        }.items(),
    )

    # ============================================================
    # 11. RViz2
    # ============================================================

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{
            "use_sim_time": True,
        }],
    )

    # ============================================================
    # 12. Funções de transição
    # ============================================================

    def apos_gerar_urdf(event, context):

        if event.returncode != 0:
            return [
                EmitEvent(
                    event=Shutdown(
                        reason="Falha ao gerar o URDF."
                    )
                )
            ]

        return [
            LogInfo(msg="URDF gerado. Inserindo robô..."),
            spawn_robot,
        ]

    def apos_inserir_robo(event, context):

        if event.returncode != 0:
            return [
                EmitEvent(
                    event=Shutdown(
                        reason="Falha ao inserir o robô."
                    )
                )
            ]

        return [
            LogInfo(
                msg="Robô inserido. Verificando sensores e TF..."
            ),
            pre_slam_monitor,
        ]

    def apos_monitor_pre_slam(event, context):

        if event.returncode != 0:
            return [
                EmitEvent(
                    event=Shutdown(
                        reason="Monitor pré-SLAM terminou com erro."
                    )
                )
            ]

        return [
            LogInfo(
                msg="Sensores e TF disponíveis. Iniciando SLAM..."
            ),
            #slam_launch,
            rviz,
        ]

    # ============================================================
    # 13. Eventos
    # ============================================================

    spawn_after_xacro = RegisterEventHandler(
        OnProcessExit(
            target_action=generate_urdf,
            on_exit=apos_gerar_urdf,
        )
    )

    monitor_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=apos_inserir_robo,
        )
    )

    slam_after_monitor = RegisterEventHandler(
        OnProcessExit(
            target_action=pre_slam_monitor,
            on_exit=apos_monitor_pre_slam,
        )
    )

    # ============================================================
    # 14. Launch Description
    # ============================================================

    return LaunchDescription([

        gazebo,

        robot_state_publisher,

        sensor_bridge,

        tf_bridge,

        ground_truth_odom_node,

        spawn_after_xacro,

        monitor_after_spawn,

        slam_after_monitor,

        generate_urdf,

    ])
