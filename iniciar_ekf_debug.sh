#!/usr/bin/env bash
# Inicializa EKF-SLAM sobre uma simulacao Gazebo ja em execucao.
# Ordem: simulation.launch.py (externo) -> detector -> EKF -> Map Builder -> Nav2.
# Uso: ./iniciar_ekf_debug.sh [--sem-nav2] [--help]
set -uo pipefail

NAV2=true
for arg in "$@"; do
  case "$arg" in
    --sem-nav2) NAV2=false ;;
    -h|--help)
      printf 'Uso: %s [--sem-nav2]\n' "$0"
      printf 'Inicie antes: ros2 launch tcc_slam_bringup simulation.launch.py\n'
      exit 0 ;;
    *) printf '[ERRO] Opcao desconhecida: %s\n' "$arg" >&2; exit 2 ;;
  esac
done

TCC="$HOME/tcc_slam_ws"
MRPT="$HOME/mrpt_ws_v3"
SCRIPTS="$TCC/src/tcc_slam_bringup/scripts"
INI="$MRPT/testes/ekf/ekf_slam.ini"
LOG_ROOT="$TCC/logs/ekf_debug"
LOG_DIR="$LOG_ROOT/$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$LOG_DIR" || exit 1
printf '%s\n' "$LOG_DIR" > "$LOG_ROOT/ultimo_log.txt"

for file in \
  /opt/ros/lyrical/setup.bash \
  "$MRPT/install/local_setup.bash" \
  "$TCC/install/local_setup.bash" \
  "$SCRIPTS/landmark_detector.py" \
  "$SCRIPTS/ekf_map_builder.py" \
  "$INI"; do
  if [[ ! -f "$file" ]]; then
    printf '[ERRO] Arquivo ausente: %s\n' "$file" >&2
    exit 1
  fi
done

# setup.bash de ROS usa variaveis que podem estar indefinidas.
set +u
source /opt/ros/lyrical/setup.bash
source "$MRPT/install/local_setup.bash"
source "$TCC/install/local_setup.bash"
set -u

if ! command -v setsid >/dev/null; then
  echo '[ERRO] Comando setsid nao encontrado.' >&2
  exit 1
fi

declare -a PIDS=()
declare -a NAMES=()
STOPPING=false
cleanup() {
  if [[ "$STOPPING" == true ]]; then return; fi
  STOPPING=true
  trap - EXIT INT TERM
  echo
  echo '[EKF] Encerrando os componentes desta sessao...'
  local pid
  for pid in "${PIDS[@]}"; do
    kill -TERM -- "-$pid" 2>/dev/null || true
  done
  sleep 2
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -KILL -- "-$pid" 2>/dev/null || true
    fi
  done
  for pid in "${PIDS[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
  echo "[EKF] Logs preservados em: $LOG_DIR"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_component() {
  local name="$1"
  shift
  echo "[EKF] Iniciando $name..."
  setsid "$@" >"$LOG_DIR/$name.log" 2>&1 &
  local pid=$!
  PIDS+=("$pid")
  NAMES+=("$name")
  echo "      PID $pid | $LOG_DIR/$name.log"
}

check_components() {
  local i
  for i in "${!PIDS[@]}"; do
    if ! kill -0 "${PIDS[$i]}" 2>/dev/null; then
      echo "[ERRO] ${NAMES[$i]} terminou. Ultimas mensagens:" >&2
      tail -n 50 "$LOG_DIR/${NAMES[$i]}.log" >&2 || true
      return 1
    fi
  done
}

publisher_exists() {
  local topic="$1"
  local info
  info=$(timeout 4 ros2 topic info "$topic" 2>/dev/null || true)
  grep -Eq 'Publisher count: [1-9][0-9]*' <<< "$info"
}

wait_topic() {
  local topic="$1" timeout_s="$2"
  local i
  echo "[EKF] Verificando publicador de $topic (limite ${timeout_s}s)..."
  for ((i=0; i<timeout_s; i+=2)); do
    check_components || return 1
    if publisher_exists "$topic"; then
      echo "[OK] $topic possui publicador."
      return 0
    fi
    sleep 2
  done
  echo "[ERRO] Sem publicador em $topic. Verifique o Gazebo e as bridges." >&2
  return 1
}

# Checagens que nao alteram outros processos ROS.
if ros2 node list 2>/dev/null | grep -Eq '^/(mrpt_ekf_slam_2d|landmark_detector|ekf_map_builder)$'; then
  echo '[ERRO] Um componente EKF ja esta ativo. Encerre a sessao anterior primeiro.' >&2
  exit 1
fi
if publisher_exists /map; then
  echo '[ERRO] /map ja possui publicador. Evite misturar mapas de algoritmos diferentes.' >&2
  ros2 topic info /map -v 2>/dev/null | head -n 25 >&2 || true
  exit 1
fi

# O simulation.launch.py e o RViz2 SAO EXTERNOS a este script.
wait_topic /clock 20 || exit 1
wait_topic /scan 20 || exit 1
wait_topic /odom 20 || exit 1

start_component detector python3 "$SCRIPTS/landmark_detector.py" \
  --ros-args -p use_sim_time:=true
sleep 1
check_components || exit 1

start_component ekf ros2 launch mrpt_ekf_slam_2d ekf_slam_2d.launch.py \
  use_sim_time:=true \
  "ini_filename:=$INI" \
  global_frame_id:=map \
  odom_frame_id:=odom \
  base_frame_id:=base_link \
  sensor_source:=/landmark_observations \
  launch_rviz:=false
sleep 2
check_components || exit 1

start_component map_builder python3 "$SCRIPTS/ekf_map_builder.py" \
  --ros-args -p use_sim_time:=true
sleep 1
check_components || exit 1

if "$NAV2"; then
  echo '[EKF] Iniciando Nav2. Seu readiness_monitor aguardara /map e TF.'
  start_component nav2 ros2 launch tcc_slam_bringup navigation.launch.py \
    start_simulation:=false bootstrap_graphslam:=false
else
  echo '[EKF] Nav2 desativado (--sem-nav2).'
fi

cat <<MSG

[EKF] Processos iniciados. A simulacao e o RViz2 permanecem no terminal externo.
[EKF] Logs desta sessao: $LOG_DIR
[EKF] Para ver os logs, em OUTRO terminal:
      tail -f "$LOG_DIR/ekf.log" "$LOG_DIR/map_builder.log"
[EKF] O mapa so fica disponivel apos TF e scans validos.
[EKF] Para encerrar os componentes EKF/Nav2: Ctrl+C neste terminal.
[EKF] Isso nao encerrara o simulation.launch.py iniciado separadamente.

MSG

while true; do
  check_components || exit 1
  sleep 2
done
