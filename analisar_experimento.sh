#!/usr/bin/env bash
# TCC_ROUTE_END_V1
set -euo pipefail

if [[ "$#" -ne 1 || ! "$1" =~ ^[0-9]+$ ]]; then
  echo "Uso: $0 NUMERO" >&2
  exit 2
fi
NUMERO="$1"
WORKSPACE="$HOME/tcc_slam_ws"
RESOLVIDO="$(python3 "$WORKSPACE/experimento_paths.py" "$NUMERO" --workspace "$WORKSPACE")" || exit 1
mapfile -t PASTAS <<< "$RESOLVIDO"
[[ "${#PASTAS[@]}" -eq 2 ]] || { echo "Falha ao localizar diretórios" >&2; exit 1; }
DIR="${PASTAS[0]}"
RESULTADOS="${PASTAS[1]}"
MANIFEST="$DIR/manifest.json"
BAG="$DIR/dados/ground_truth"

[[ -f "$MANIFEST" ]] || {
  echo "Manifest ausente: $MANIFEST" >&2
  echo "Não será inferida duração ou método; para dados antigos, preencha manifest.json." >&2
  exit 1
}
[[ -f "$BAG/metadata.yaml" ]] || { echo "Rosbag ausente: $BAG" >&2; exit 1; }

# Confere os metadados do experimento original, sem valores inventados.
# Lemos metodo, duracao (quando aplicavel) e modo registrados no manifest.
METADADOS="$(python3 - "$MANIFEST" <<'PYJSON'
import json, sys, math
with open(sys.argv[1],encoding='utf-8') as f:
    m=json.load(f)
metodo = m.get('slam_method')
modo = m.get('modo_experimento') or 'fronteiras'
if metodo not in ('toolbox','graph','rbpf','ekf'):
    sys.exit('Metodo ausente/invalido no manifest.json')
if modo not in ('fronteiras', 'rota_fixa'):
    sys.exit('Modo de movimentacao invalido no manifest.json')
if modo == 'fronteiras':
    try: duracao = float(m.get('duracao_configurada_s'))
    except (TypeError, ValueError): sys.exit('Duracao invalida no manifest')
    if not math.isfinite(duracao) or duracao <= 0:
        sys.exit('Duracao deve ser positiva no modo fronteiras')
else:
    duracao = 'auto'
print(metodo)
print(duracao)
print(modo)
PYJSON
)" || exit 1
mapfile -t PARAMETROS <<< "$METADADOS"
[[ "${#PARAMETROS[@]}" -eq 3 ]] || { echo "Manifest invalido" >&2; exit 1; }
METODO="${PARAMETROS[0]}"
DURACAO="${PARAMETROS[1]}"
MODO="${PARAMETROS[2]}"

# setup.bash de ROS pode usar variáveis não inicializadas; evite 'nounset' nesta região.
set +u
source /opt/ros/lyrical/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u
mkdir -p "$RESULTADOS"
printf '\nANALISE | id=%s | pasta=%s | metodo=%s | modo=%s | duracao=%s\n' "$NUMERO" "$(basename "$DIR")" "$METODO" "$MODO" "$DURACAO"

ARGUMENTOS=("$BAG" --storage-id mcap --metodo "$METODO" --movimentacao "$MODO" --alinhamento inicial
    --saida-trajetoria "$RESULTADOS/trajetoria.csv"
    --saida-metricas "$RESULTADOS/metricas.csv")
if [[ "$MODO" == "fronteiras" ]]; then
    ARGUMENTOS+=(--duracao "$DURACAO")
fi
ros2 run tcc_slam_analysis analisar_experimento "${ARGUMENTOS[@]}"

printf '\nRelatorio: %s/relatorio.html\nMapa: %s/mapa_final.png\n' "$RESULTADOS" "$RESULTADOS"
