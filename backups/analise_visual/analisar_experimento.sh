#!/usr/bin/env bash
set -eo pipefail

if [ "$#" -lt 2 ] || [ "$#" -gt 3 ]; then
    echo "Uso: $0 NUMERO DURACAO [toolbox|graph|rbpf|ekf]"
    exit 1
fi
NUMERO="$1"
DURACAO="$2"
METODO="${3:-nao_informado}"

[[ "$NUMERO" =~ ^[0-9]+$ ]] || { echo "Numero invalido"; exit 1; }
[[ "$DURACAO" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "Duracao invalida"; exit 1; }
awk -v d="$DURACAO" 'BEGIN { exit !(d > 0) }' || { echo "Duracao deve ser positiva"; exit 1; }
case "$METODO" in toolbox|graph|rbpf|ekf|nao_informado) ;; *) echo "Metodo invalido: $METODO"; exit 1 ;; esac
ID=$(printf "%03d" "$((10#$NUMERO))")
WORKSPACE="$HOME/tcc_slam_ws"
BAG="$WORKSPACE/bags/experimento_${ID}/dados/ground_truth"
RESULTADOS="$WORKSPACE/results/experimento_${ID}"

# source ROS 2: protecao para quem usa set -u no terminal chamador
source /opt/ros/lyrical/setup.bash
source "$WORKSPACE/install/setup.bash"
[[ -f "$BAG/metadata.yaml" ]] || { echo "Rosbag ausente: $BAG"; exit 1; }
mkdir -p "$RESULTADOS"
printf '\nANALISE | experimento=%s | metodo=%s | duracao=%ss\n' "$ID" "$METODO" "$DURACAO"

ros2 run tcc_slam_analysis analisar_experimento \
    "$BAG" \
    --storage-id mcap \
    --duracao "$DURACAO" \
    --metodo "$METODO" \
    --alinhamento inicial \
    --saida-trajetoria "$RESULTADOS/trajetoria.csv" \
    --saida-metricas "$RESULTADOS/metricas.csv"

printf '\nAnalise concluida: %s (%s) -> %s\n' "$ID" "$METODO" "$RESULTADOS"
