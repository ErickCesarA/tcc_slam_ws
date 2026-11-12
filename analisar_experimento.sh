#!/usr/bin/env bash
set -eo pipefail

if [[ "$#" -ne 1 || ! "$1" =~ ^[0-9]+$ ]]; then
  echo "Uso: $0 NUMERO" >&2
  exit 2
fi

NUMERO="$1"
ID=$(printf '%03d' "$((10#$NUMERO))")
WORKSPACE="$HOME/tcc_slam_ws"
DIR="$WORKSPACE/bags/experimento_${ID}"
MANIFEST="$DIR/manifest.json"
BAG="$DIR/dados/ground_truth"
RESULTADOS="$WORKSPACE/results/experimento_${ID}"

if [[ ! -f "$MANIFEST" ]]; then
  echo "ERRO: manifest.json ausente: $MANIFEST" >&2
  echo "O arquivo registra a duracao e o metodo originais. Nao sera assumida uma duracao padrao." >&2
  echo "Para uma gravacao antiga, crie manifest.json com slam_method e duracao_configurada_s corretos." >&2
  exit 1
fi
[[ -f "$BAG/metadata.yaml" ]] || { echo "Rosbag ausente: $BAG" >&2; exit 1; }

# Lemos manifest com Python padrão, sem depender do ROS.
readarray -t PARAMETROS < <(python3 - "$MANIFEST" <<'PYJSON'
import json,sys,math
with open(sys.argv[1],encoding="utf-8") as f:
    m=json.load(f)
metodo=m.get("slam_method")
duracao=m.get("duracao_configurada_s")
if metodo not in ("toolbox","graph","rbpf","ekf"):
    sys.exit("Metodo ausente/invalido no manifest.json")
try:
    duracao=float(duracao)
except (TypeError, ValueError):
    sys.exit("Duracao ausente/invalida no manifest.json")
if not math.isfinite(duracao) or duracao<=0:
    sys.exit("Duracao deve ser positiva e finita")
print(metodo)
print(duracao)
PYJSON
)
if [[ "${#PARAMETROS[@]}" -ne 2 ]]; then
  echo "ERRO: manifest.json sem metodo/duracao validos." >&2
  exit 1
fi
METODO="${PARAMETROS[0]}"
DURACAO="${PARAMETROS[1]}"

source /opt/ros/lyrical/setup.bash
source "$WORKSPACE/install/setup.bash"
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

printf '\nRelatorio: %s/relatorio.html\nMapa: %s/mapa_final.png\n' "$RESULTADOS" "$RESULTADOS"
