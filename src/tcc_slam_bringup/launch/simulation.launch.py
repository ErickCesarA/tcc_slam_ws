import os
import sys

from launch import LaunchDescription

from launch.actions import (
    ExecuteProcess,
    RegisterEventHandler,
    IncludeLaunchDescription,
    LogInfo,
    EmitEvent,
    OpaqueFunction,
    DeclareLaunchArgument,
)

from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory


# ============================================================
# 1. CADASTRO DOS MAPAS
# ============================================================

MAPAS = {
    "1": {
        "arquivo": "tcc_world.sdf",
        "world_name": "tcc_world",
    },

    "2": {
        "arquivo": "tcc_world_2.sdf",
        "world_name": "tcc_world_2",
    },
}


def iniciar_simulacao(context):

    # ========================================================
    # 2. IDENTIFICAR O MAPA SELECIONADO
    # ========================================================

    mapa_id = LaunchConfiguration("map").perform(context).strip()

    if mapa_id not in MAPAS:

        mapas_disponiveis = ", ".join(MAPAS.keys())

        return [
            LogInfo(
                msg=(
                    f"ERRO: mapa '{mapa_id}' nao cadastrado. "
                    f"Mapas disponiveis: {mapas_disponiveis}"
                )
            ),
            EmitEvent(
                event=Shutdown(
                    reason="Mapa desconhecido."
                )
            ),
        ]

    mapa_config = MAPAS[mapa_id]

    # ========================================================
    # 3. LOCALIZAR OS PACOTES E ARQUIVOS
    # ========================================================

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
        mapa_config["arquivo"],
    )

    world_name = mapa_config["world_name"]

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

    # ========================================================
    # 4. VALIDAR O ARQUIVO DO MUNDO
    # ========================================================

    if not os.path.isfile(world_file):

        return [
            LogInfo(
                msg=f"ERRO: arquivo do mapa nao encontrado: {world_file}"
            ),
            EmitEvent(
                event=Shutdown(
                    reason="Arquivo SDF inexistente."
                )
            ),
        ]

    # ========================================================
    # 5. INFORMAR O MAPA SELECIONADO
    # ========================================================

    print("\n")
    print("=" * 60)
    print(f" MAPA SELECIONADO: {mapa_id}")
    print(f" ARQUIVO SDF: {mapa_config['arquivo']}")
    print(f" WORLD GAZEBO: {world_name}")
    print("=" * 60)
    print("\n")

    # ========================================================
    # 6. INICIAR O GAZEBO
    # ========================================================

    gazebo = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-r",
            world_file,
        ],
        output="screen",
    )

    # ========================================================
    # 7. GERAR URDF
    # ========================================================

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

    # ========================================================
    # 8. ROBOT STATE PUBLISHER
    # ========================================================

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

    # ========================================================
    # 9. BRIDGE DOS SENSORES
    # ========================================================

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

    # ========================================================
    # 10. BRIDGE TF
    # ========================================================

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

    # ========================================================
    # 11. GROUND TRUTH
    # ========================================================

    ground_truth_topic = (
        f"/world/{world_name}/pose/info"
    )

    ground_truth_odom_node = Node(
        package="tcc_slam_ground_truth",
        executable="ground_truth_odom",
        name="ground_truth_odom",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"gazebo_topic": ground_truth_topic},
            {"model_name": "tcc_robot"},
            {"world_frame": "world"},
            {"child_frame": "tcc_robot"},
        ],
    )

    # ========================================================
    # 12. INSERIR O ROBÔ
    # ========================================================

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot",
        output="screen",
        arguments=[
            "-world",
            world_name,
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

    # ========================================================
    # 13. MONITOR DE PRONTIDAO PRE-SLAM
    # ========================================================

    pre_slam_monitor = ExecuteProcess(
        cmd=[
            sys.executable,
            monitor_file,
            "--mode",
            "pre_slam",
        ],
        output="screen",
    )

    # ========================================================
    # 14. SLAM TOOLBOX
    # ========================================================

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

    # ========================================================
    # 15. RVIZ2
    # ========================================================

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{
            "use_sim_time": True,
        }],
    )

    # ========================================================
    # 16. FUNCOES DE TRANSICAO
    # ========================================================

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
            LogInfo(
                msg="URDF gerado. Inserindo robo..."
            ),
            spawn_robot,
        ]

    def apos_inserir_robo(event, context):

        if event.returncode != 0:

            return [
                EmitEvent(
                    event=Shutdown(
                        reason="Falha ao inserir o robo."
                    )
                )
            ]

        return [
            LogInfo(
                msg="Robo inserido. Verificando sensores e TF..."
            ),
            pre_slam_monitor,
        ]

    def apos_monitor_pre_slam(event, context):

        if event.returncode != 0:

            return [
                EmitEvent(
                    event=Shutdown(
                        reason="Monitor pre-SLAM terminou com erro."
                    )
                )
            ]

        return [
            LogInfo(
                msg="Sensores e TF disponiveis. Inicializacao concluida."
            ),
            # slam_launch e iniciado pelo navigation.launch.py
            rviz,
        ]

    # ========================================================
    # 17. EVENTOS
    # ========================================================

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

    # ========================================================
    # 18. INICIALIZACAO DA SIMULACAO
    # ========================================================

    return [

        LogInfo(
            msg=f"Iniciando simulacao com o mapa {mapa_id}: {world_name}"
        ),

        gazebo,

        robot_state_publisher,

        sensor_bridge,

        tf_bridge,

        ground_truth_odom_node,

        spawn_after_xacro,

        monitor_after_spawn,

        slam_after_monitor,

        generate_urdf,

    ]


# ============================================================
# 19. LAUNCH DESCRIPTION
# ============================================================

def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            "map",
            default_value="1",
            description="Identificador do mapa a ser utilizado",
        ),

        OpaqueFunction(
            function=iniciar_simulacao
        ),

    ])
