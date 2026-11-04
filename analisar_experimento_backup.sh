#!/usr/bin/env bash

set -e

if [ -z "$1" ]; then
    echo "Uso: $0 NUMERO_DO_EXPERIMENTO"
    exit 1
fi

NUMERO="$1"

if ! [[ "$NUMERO" =~ ^[0-9]+$ ]]; then
    echo "Erro: informe um número de experimento válido."
    exit 1
fi

ID=$(printf "%03d" "$((10#$NUMERO))")

BAG="$HOME/tcc_slam_ws/bags/experimento_${ID}/dados/ground_truth"
RESULTADOS="$HOME/tcc_slam_ws/results/experimento_${ID}"

source /opt/ros/lyrical/setup.bash
source "$HOME/tcc_slam_ws/install/setup.bash"

if [ ! -d "$BAG" ]; then
    echo "Erro: rosbag não encontrada em:"
    echo "$BAG"
    exit 1
fi

mkdir -p "$RESULTADOS"

ros2 run tcc_slam_analysis analisar_experimento \
    "$BAG" \
    --referencia /ground_truth/odom \
    --estimada /odom \
    --saida-trajetoria "$RESULTADOS/trajetoria.csv" \
    --saida-metricas "$RESULTADOS/metricas.csv"

echo
echo "Análise do experimento $ID concluída."
echo "Resultados em: $RESULTADOS"
