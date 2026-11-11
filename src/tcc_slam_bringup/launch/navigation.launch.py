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
    OpaqueFunction,
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

from ament_index_python.packages import (
    get_package_share_directory,
)


def generate_launch_description():

    # ============================================================
    # 1. CONFIGURACOES
    # ============================================================

    use_sim_time = LaunchConfiguration("use_sim_time")

    slam_method = LaunchConfiguration("slam_method")

    bringup_share = get_package_share_directory(
        "tcc_slam_bringup"
    )

    monitor_file = os.path.join(
        bringup_share,
        "launch",
        "readiness_monitor.py",
    )

    bootstrap_file = os.path.join(bringup_share, "scripts", "bootstrap_graphslam.py")
    landmark_detector_file = os.path.join(bringup_share, "scripts", "landmark_detector.py")
    ekf_map_builder_file = os.path.join(bringup_share, "scripts", "ekf_map_builder.py")
    mrpt_config = os.path.join(bringup_share, "config", "mrpt")

    nav2_params = PathJoinSubstitution([
        FindPackageShare("tcc_slam_bringup"),
        "config",
        "nav2_params.yaml",
    ])

    # ============================================================
    # 2. SIMULACAO E METODOS SLAM
    # ============================================================

    # A simulacao e compartilhada por todos os metodos de SLAM.
    # Iniciada uma unica vez neste launch; nao iniciar simulation.launch.py a parte.
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "simulation.launch.py")
        ),
    )

    toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("slam_toolbox"), "launch", "online_sync_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": os.path.join(bringup_share, "config", "slam_toolbox.yaml"),
        }.items(),
    )

    graph_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("mrpt_graphslam_2d"), "launch", "graphslam.launch.py")
        ),
        launch_arguments={
            "config_file": os.path.join(get_package_share_directory("mrpt_graphslam_2d"), "config", "ros_laser_odometry.ini"),
            "launch_rviz": "false",
        }.items(),
    )

    rbpf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("mrpt_rbpf_slam"), "launch", "rbpf_slam.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "ini_filename": os.path.join(mrpt_config, "rbpf_slam.ini"),
            "config_file": os.path.join(mrpt_config, "rbpf_params.yaml"),
            "global_frame_id": "map",
            "odom_frame_id": "odom",
            "base_frame_id": "base_link",
            "sensor_source": "/scan",
            "launch_rviz": "false",
        }.items(),
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("mrpt_ekf_slam_2d"), "launch", "ekf_slam_2d.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "ini_filename": os.path.join(mrpt_config, "ekf_slam.ini"),
            "global_frame_id": "map",
            "odom_frame_id": "odom",
            "base_frame_id": "base_link",
            "sensor_source": "/landmark_observations",
            "launch_rviz": "false",
        }.items(),
    )

    landmark_detector = ExecuteProcess(
        cmd=[sys.executable, landmark_detector_file, "--ros-args", "-p", "use_sim_time:=true"],
        output="screen", name="landmark_detector",
    )

    ekf_map_builder = ExecuteProcess(
        cmd=[sys.executable, ekf_map_builder_file, "--ros-args", "-p", "use_sim_time:=true"],
        output="screen", name="ekf_map_builder",
    )

    # ============================================================
    # 3. BOOTSTRAP GRAPHSLAM
    # ============================================================

    bootstrap_process = ExecuteProcess(
        cmd=[
            sys.executable,
            bootstrap_file,
        ],
        output="screen",
        name="bootstrap_graphslam",
    )

    # ============================================================
    # 4. MONITOR PRE-NAV2
    # ============================================================

    pre_nav_monitor = ExecuteProcess(
        cmd=[
            sys.executable,
            monitor_file,
            "--mode",
            "pre_nav",
        ],
        output="screen",
        name="readiness_monitor_pre_nav",
    )

    # ============================================================
    # 5. SERVIDORES NAV2
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
    # 6. LIFECYCLE MANAGER
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
    # 7. CALLBACK: FINALIZACAO DO BOOTSTRAP
    # ============================================================

    def apos_bootstrap(event, context):

        # O script deve retornar codigo 0 em caso de sucesso.
        # Qualquer outro codigo interrompe a inicializacao.

        if event.returncode != 0:

            return [
                LogInfo(
                    msg=(
                        "ERRO: Bootstrap GraphSLAM falhou. "
                        "Nav2 nao sera iniciado."
                    )
                ),

                EmitEvent(
                    event=Shutdown(
                        reason="Falha no bootstrap GraphSLAM."
                    )
                ),
            ]

        return [
            LogInfo(
                msg=(
                    "Bootstrap GraphSLAM concluido. "
                    "Fronteira encontrada e robo parado."
                )
            ),

            LogInfo(
                msg="Iniciando monitor pre-Nav2..."
            ),

            pre_nav_monitor,
        ]

    bootstrap_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=bootstrap_process,
            on_exit=apos_bootstrap,
        )
    )

    # ============================================================
    # 8. CALLBACK: FINALIZACAO DO MONITOR PRE-NAV2
    # ============================================================

    def apos_monitor_pre_nav(event, context):

        if event.returncode != 0:

            return [
                LogInfo(
                    msg=(
                        "ERRO: Monitor pre-Nav2 falhou. "
                        "Inicializacao interrompida."
                    )
                ),

                EmitEvent(
                    event=Shutdown(
                        reason="Monitor pre-Nav2 terminou com erro."
                    )
                ),
            ]

        return [
            LogInfo(
                msg=(
                    "Mapa, relogio e TF verificados. "
                    "Iniciando Nav2..."
                )
            ),

            navigation_launch,

            lifecycle_manager,
        ]

    monitor_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=pre_nav_monitor,
            on_exit=apos_monitor_pre_nav,
        )
    )

    # ============================================================
    # 9. SELECAO DO METODO E FLUXO DE INICIALIZACAO
    # ============================================================

    def selecionar_inicializacao(context):
        metodo = slam_method.perform(context).strip().lower()
        arquivos = {
            "toolbox": [],
            "graph": [],
            "rbpf": [os.path.join(mrpt_config, "rbpf_slam.ini"),
                     os.path.join(mrpt_config, "rbpf_params.yaml")],
            "ekf": [os.path.join(mrpt_config, "ekf_slam.ini")],
        }
        if metodo not in arquivos:
            return [
                LogInfo(msg=f"ERRO: slam_method={metodo!r} invalido. Use toolbox, graph, rbpf ou ekf."),
                EmitEvent(event=Shutdown(reason="Metodo SLAM desconhecido.")),
            ]
        ausentes = [arquivo for arquivo in arquivos[metodo] if not os.path.isfile(arquivo)]
        if ausentes:
            return [
                LogInfo(msg="ERRO: configuracoes MRPT ausentes: " + ", ".join(ausentes)),
                EmitEvent(event=Shutdown(reason="Configuracao de SLAM ausente.")),
            ]

        # Importante: apenas o metodo selecionado deve publicar /map e map->odom.
        if metodo == "toolbox":
            return [LogInfo(msg="SLAM Toolbox selecionado"), toolbox_launch, pre_nav_monitor]
        if metodo == "rbpf":
            return [LogInfo(msg="MRPT RBPF-SLAM selecionado"), rbpf_launch, pre_nav_monitor]
        if metodo == "ekf":
            return [LogInfo(msg="MRPT EKF-SLAM selecionado"),
                    landmark_detector, ekf_launch, ekf_map_builder, pre_nav_monitor]
        # O GraphSLAM utiliza o bootstrap para descobrir as primeiras fronteiras
        # antes do Nav2, preservando o fluxo experimental anterior.
        return [LogInfo(msg="MRPT GraphSLAM selecionado"),
                graph_launch, bootstrap_process]

    iniciar_fluxo = OpaqueFunction(function=selecionar_inicializacao)

    # ============================================================
    # 10. LAUNCH DESCRIPTION
    # ============================================================

    return LaunchDescription([

        # --------------------------------------------------------
        # Argumentos
        # --------------------------------------------------------

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Utilizar relogio da simulacao",
        ),

        DeclareLaunchArgument(
            "slam_method",
            default_value="toolbox",
            description="Metodo de SLAM: toolbox, graph, rbpf ou ekf",
        ),

        # --------------------------------------------------------
        # Registrar eventos
        # --------------------------------------------------------

        bootstrap_handler,

        monitor_handler,

        # --------------------------------------------------------
        # Selecionar metodo e fluxo de inicializacao
        # --------------------------------------------------------

        simulation_launch,

        iniciar_fluxo,

    ])
