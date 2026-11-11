#!/usr/bin/env bash

set -eo pipefail

# ============================================================
# VALIDAÇÃO DOS ARGUMENTOS
# ============================================================

if [ $# -lt 1 ]; then
    echo "Uso: $0 NUMERO_DO_EXPERIMENTO [DURACAO]"
    echo
    echo "Exemplos:"
    echo "  $0 4 60"
    echo "  $0 5 600"
    echo "  $0 5"
    exit 1
fi

NUMERO="$1"
DURACAO="${2:-600}"

if ! [[ "$NUMERO" =~ ^[0-9]+$ ]]; then
    echo "Erro: número do experimento inválido."
    exit 1
fi

if ! [[ "$DURACAO" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "Erro: duração inválida."
    exit 1
fi

if ! awk -v d="$DURACAO" 'BEGIN {exit !(d > 0)}'; then
    echo "Erro: a duração deve ser maior que zero."
    exit 1
fi

# ============================================================
# IDENTIFICAÇÃO DO EXPERIMENTO
# ============================================================

ID=$(printf "%03d" "$((10#$NUMERO))")

WORKSPACE="$HOME/tcc_slam_ws"

BAG="$WORKSPACE/bags/experimento_${ID}/dados/ground_truth"

RESULTADOS="$WORKSPACE/results/experimento_${ID}"

# ============================================================
# CONFIGURAÇÃO DO ROS 2
# ============================================================

source /opt/ros/lyrical/setup.bash
source "$WORKSPACE/install/setup.bash"

# ============================================================
# VERIFICAÇÃO DA ROSBAG
# ============================================================

if [ ! -d "$BAG" ]; then
    echo "Erro: rosbag não encontrada:"
    echo "$BAG"
    exit 1
fi

if [ ! -f "$BAG/metadata.yaml" ]; then
    echo "Erro: metadata.yaml não encontrado na rosbag."
    exit 1
fi

mkdir -p "$RESULTADOS"

# ============================================================
# INFORMAÇÕES DO EXPERIMENTO
# ============================================================

echo
echo "=========================================="
echo "ANÁLISE DE EXPERIMENTO SLAM"
echo "=========================================="
echo "Experimento: $ID"
echo "Duração configurada: $DURACAO segundos"
echo "Rosbag: $BAG"
echo "Resultados: $RESULTADOS"
echo "=========================================="
echo

# ============================================================
# EXECUÇÃO DA ANÁLISE
# ============================================================

ros2 run tcc_slam_analysis analisar_experimento \
    "$BAG" \
    --storage-id mcap \
    --duracao "$DURACAO" \
    --alinhamento inicial \
    --saida-trajetoria "$RESULTADOS/trajetoria.csv" \
    --saida-metricas "$RESULTADOS/metricas.csv"

# ============================================================
# FINALIZAÇÃO
# ============================================================

echo
echo "=========================================="
echo "ANÁLISE CONCLUÍDA"
echo "=========================================="
echo "Experimento: $ID"
echo "Resultados disponíveis em:"
echo "$RESULTADOS"
echo "=========================================="
