#!/usr/bin/env python3
# TCC_ROUTE_END_V1
"""Convenções de diretórios de experimentos; suporta formato legado.

Formato novo: exp_17_mtd_graph_time_180_map_2
Legado:      experimento_017
"""
from __future__ import annotations
import argparse
import math
from pathlib import Path
import re

METODOS = {"toolbox", "graph", "rbpf", "ekf"}
NOVO = re.compile(r"^exp_(\d+)_mtd_(toolbox|graph|rbpf|ekf)_time_(auto|[0-9]+(?:p[0-9]+)?)_map_(\d+)$")
ANTIGO = re.compile(r"^experimento_(\d+)$")


def nome_experimento(numero: int, metodo: str, duracao: float, mapa: int) -> str:
    if numero < 0 or metodo not in METODOS or mapa < 1:
        raise ValueError('Número >= 0, mapa >= 1 e método válido são obrigatórios.')
    # Rota fixa: tempo real e conhecido apenas ao completar todos os pontos.
    if duracao is None:
        tempo = 'auto'
    else:
        if not math.isfinite(duracao) or duracao <= 0:
            raise ValueError('Duração deve ser positiva e finita.')
        if float(duracao).is_integer():
            tempo = str(int(duracao))
        else:
            tempo = f'{duracao:.6f}'.rstrip('0').rstrip('.').replace('.', 'p')
    return f'exp_{numero}_mtd_{metodo}_time_{tempo}_map_{mapa}'


def id_da_pasta(nome: str) -> int | None:
    match = NOVO.fullmatch(nome) or ANTIGO.fullmatch(nome)
    return int(match.group(1)) if match else None


def localizar_pasta(basedir: Path, numero: int, obrigatoria: bool = True) -> Path | None:
    """Procura ID exato em nomes novos e antigos; detecta ambiguidades."""
    candidatos = []
    if basedir.is_dir():
        for p in basedir.iterdir():
            if p.is_dir() and id_da_pasta(p.name) == numero:
                candidatos.append(p)
    if len(candidatos) > 1:
        raise RuntimeError(f'ID {numero} ambíguo em {basedir}: '
                           + ', '.join(sorted(x.name for x in candidatos)))
    if not candidatos and obrigatoria:
        raise FileNotFoundError(f'Experimento {numero} não encontrado em {basedir}')
    return candidatos[0] if candidatos else None


def localizar_experimento(ws: Path, numero: int):
    """Retorna (bagdir, resultsdir), usando mesmo nome para novos registros."""
    bag = localizar_pasta(ws / 'bags', numero)
    resultados = localizar_pasta(ws / 'results', numero, obrigatoria=False)
    if resultados is None:
        resultados = ws / 'results' / bag.name
    if resultados.name != bag.name and not (
        ANTIGO.fullmatch(resultados.name) and ANTIGO.fullmatch(bag.name)
    ):
        raise RuntimeError(f'Pastas bag/results incompatíveis: {bag.name} / {resultados.name}')
    return bag, resultados


def checar_id_disponivel(ws: Path, numero: int):
    if localizar_pasta(ws / 'bags', numero, obrigatoria=False) is not None:
        raise FileExistsError(f'Já existe experimento {numero} em bags/.')
    if localizar_pasta(ws / 'results', numero, obrigatoria=False) is not None:
        raise FileExistsError(f'Já existe experimento {numero} em results/.')


def main():
    ap = argparse.ArgumentParser(description='Localiza experimento pelo ID, com compatibilidade antiga.')
    ap.add_argument('numero', type=int)
    ap.add_argument('--workspace', type=Path, default=Path.home() / 'tcc_slam_ws')
    args = ap.parse_args()
    if args.numero < 0:
        ap.error('Número não pode ser negativo.')
    try:
        bag, resultados = localizar_experimento(args.workspace.expanduser(), args.numero)
    except (OSError, RuntimeError, ValueError) as err:
        ap.exit(1, f'ERRO: {err}\n')
    print(bag)
    print(resultados)


if __name__ == '__main__':
    main()
