#!/usr/bin/env python3
"""Painel de repetibilidade para o comparar_experimento.py existente.

Não compara métodos distintos e não altera as métricas originais. O painel só é
produzido quando as execuções representam a mesma condição experimental.
"""
from __future__ import annotations

import csv
from html import escape
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


INDICADORES_FRONTEIRAS = [
    ('slam_ATE_m_RMSE', 'ATE translacional (RMSE)', 'm'),
    ('slam_ATE_angular_graus_RMSE', 'ATE angular (RMSE)', 'graus'),
    ('slam_RPE_1s_translacao_m_RMSE', 'RPE translacional (1 s)', 'm'),
    ('mapa_variacao_area_conhecida_m2', 'Expansão da área conhecida', 'm²'),
    ('mapa_taxa_media_m2_s', 'Taxa de expansão do mapa', 'm²/s'),
    ('exploracao_eficiencia_m2_por_m', 'Área por distância', 'm²/m'),
    ('exploracao_objetivos_alcancados', 'Objetivos alcançados', ''),
    ('slam_distancia_referencia_m', 'Distância percorrida (GT)', 'm'),
]
INDICADORES_ROTA = [
    ('rota_tempo_execucao_s', 'Tempo para concluir a rota', 's'),
    ('rota_distancia_real_m', 'Distância percorrida (GT)', 'm'),
    ('rota_waypoints_alcancados', 'Waypoints concluídos', ''),
    ('slam_ATE_m_RMSE', 'ATE translacional (RMSE)', 'm'),
    ('slam_ATE_angular_graus_RMSE', 'ATE angular (RMSE)', 'graus'),
    ('slam_RPE_1s_translacao_m_RMSE', 'RPE translacional (1 s)', 'm'),
    ('mapa_variacao_area_conhecida_m2', 'Expansão da área conhecida', 'm²'),
]


def numero(valor, casas=3):
    if valor is None:
        return '—'
    try:
        return f'{valor:.{casas}f}' if math.isfinite(valor) else '—'
    except (TypeError, ValueError):
        return '—'


def valor(dado):
    try:
        x = float(dado)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def nstr(dado):
    return None if dado is None else str(dado)


def verificar_equivalencia(exps):
    """Retorna (repetição_candidata, motivos, advertências).

    Exige coincidência explícita das condições e das rotas fixas. Não usa a
    duração *observada* como condição, pois ela é um resultado experimental.
    """
    if len(exps) < 2:
        return False, [], []
    # Comparações entre métodos/mapas não são 'repetições' e ficam inalteradas.
    identidades = {(e['metodo'], nstr(e['mapa_id']), e['modo']) for e in exps}
    if len(identidades) != 1:
        return False, [], []
    motivos, avisos = [], []
    metodo, mapa, modo = next(iter(identidades))
    if not mapa or mapa == 'None':
        motivos.append('O mapa não está identificado em todos os manifestos.')
    manifests = [e.get('manifest') or {} for e in exps]
    if not all(m for m in manifests):
        motivos.append('Falta manifest.json em pelo menos uma execução.')
    if modo == 'fronteiras':
        tempos = [valor(m.get('duracao_configurada_s', e['m'].get('experimento_duracao_configurada_s')))
                  for m, e in zip(manifests, exps)]
        if any(x is None for x in tempos) or max(tempos) - min(tempos) > 1e-6:
            motivos.append('A duração programada não é igual ou está ausente.')
    elif modo == 'rota_fixa':
        hashes = [m.get('rota_sha256') for m in manifests]
        if not all(hashes) or len(set(hashes)) != 1:
            motivos.append('As rotas não possuem o mesmo hash SHA-256.')
        if any(m.get('rota_concluida') is not True for m in manifests):
            motivos.append('Nem todas as rotas foram registradas como concluídas.')
    else:
        motivos.append(f'Modalidade não reconhecida: {modo}')
    perfis = [m.get('perfil_sensores') for m in manifests]
    if all(p is not None for p in perfis):
        if len(set(perfis)) != 1:
            motivos.append('Perfis de sensores diferentes.')
    else:
        avisos.append('Perfil dos sensores ausente em algum manifest.')
    # Hash de arquivos efetivamente fornecido pelo executor da etapa 1.
    hashes = [m.get('hashes_configuracao_sha256') for m in manifests]
    if all(isinstance(h, dict) and h for h in hashes):
        nomes = sorted(set.intersection(*(set(h) for h in hashes)))
        for nome in nomes:
            if len({h.get(nome) for h in hashes}) > 1:
                motivos.append(f'Arquivo de configuração diferente: {nome}.')
        if any(set(h) != set(hashes[0]) for h in hashes):
            motivos.append('Lista de arquivos com hash não é idêntica entre os manifestos.')
    else:
        avisos.append('Não há hashes completos de configuração em todos os manifestos.')
    # ATE pode ser comparado mesmo com versões diferentes, mas só atribuímos
    # consistência à mesma configuração quando não há divergência conhecida.
    commits = {m.get('git_commit') for m in manifests if m.get('git_commit')}
    if len(commits) > 1:
        avisos.append('Commits Git diferentes; verifique se alteraram componentes experimentais.')
    if any(m.get('git_alteracoes_nao_commitadas') is True for m in manifests):
        avisos.append('Há execuções com modificações Git não commitadas.')
    return True, motivos, avisos


def avaliar(exps, destino: Path):
    """Retorna seção HTML inserida no comparador. Salva CSV e gráfico se válido."""
    candidata, motivos, avisos = verificar_equivalencia(exps)
    if not candidata:
        return ''
    if motivos:
        lis = ''.join(f'<li>{escape(x)}</li>' for x in motivos)
        return ("<section class='warning'><h2>Repetições: condições não equivalentes</h2>"
                '<p>O mesmo método, mapa e modalidade foram informados, mas a '
                'comparação de consistência foi bloqueada:</p><ul>' + lis + '</ul></section>')
    modo = exps[0]['modo']
    escolhidas = INDICADORES_ROTA if modo == 'rota_fixa' else INDICADORES_FRONTEIRAS
    registros = []
    series = []
    for chave, titulo, unidade in escolhidas:
        pares = [(e['id'], valor(e['m'].get(chave))) for e in exps]
        validos = [(i,v) for i,v in pares if v is not None]
        if not validos:
            continue
        arr = np.array([v for _,v in validos], dtype=float)
        n = len(arr)
        media = float(arr.mean())
        desvio = float(arr.std(ddof=1)) if n >= 2 else None
        mediana = float(np.median(arr))
        mad = float(np.median(abs(arr-mediana)))
        cv = 100.0 * desvio / abs(media) if desvio is not None and abs(media) > 1e-8 else None
        mn, mx = float(arr.min()), float(arr.max())
        # Limiar de CV meramente descritivo, nunca prova de confiabilidade.
        if n < 3 or cv is None:
            classe = 'Indeterminado'
        elif cv <= 10:
            classe = 'Baixa dispersão'
        elif cv <= 25:
            classe = 'Dispersão moderada'
        else:
            classe = 'Alta dispersão'
        registros.append({
            'chave': chave, 'metrica': titulo, 'unidade': unidade,
            'n_valido': n, 'n_total': len(exps), 'media': media,
            'desvio_padrao_amostral': desvio, 'coeficiente_variacao_pct': cv,
            'mediana': mediana, 'mad': mad, 'minimo': mn, 'maximo': mx,
            'amplitude': mx-mn, 'classificacao_descritiva': classe,
        })
        series.append((titulo, pares, media, unidade))
    if not registros:
        return '<section class="warning"><h2>Repetições</h2><p>Nenhuma métrica numérica de consistência foi encontrada.</p></section>'
    with (destino / 'consistencia_repeticoes.csv').open('w', newline='', encoding='utf-8') as arq:
        wr = csv.DictWriter(arq, fieldnames=list(registros[0]))
        wr.writeheader()
        wr.writerows(registros)
    # Valores observados por repetição para reprodutibilidade.
    with (destino / 'valores_por_repeticao.csv').open('w', newline='', encoding='utf-8') as arq:
        nomes = ['experimento'] + [r['chave'] for r in registros]
        wr = csv.DictWriter(arq, fieldnames=nomes)
        wr.writeheader()
        for e in exps:
            wr.writerow({'experimento':e['id'], **{r['chave']: e['m'].get(r['chave']) for r in registros}})
    # Gráfico de resultados individuais: evita espremer indicadores de unidades distintas.
    for i, (titulo, pares, media, unidade) in enumerate(series[:4]):
        chave = registros[i]['chave']
        xs = [i for i,v in pares if v is not None]
        ys = [v for i,v in pares if v is not None]
        fig, ax = plt.subplots(figsize=(8.4, 3.6))
        ax.plot(range(len(xs)), ys, 'o-', linewidth=1.6)
        ax.axhline(media, linestyle='--', linewidth=1.2, label=f'Média {media:.4g}')
        ax.set_xticks(range(len(xs)), xs)
        ax.set_xlabel('Número do experimento')
        ax.set_ylabel(unidade)
        ax.set_title(titulo)
        ax.grid(alpha=.25)
        ax.legend()
        fig.tight_layout()
        fig.savefig(destino / 'graficos' / f'consistencia_{chave}.png', dpi=130)
        plt.close(fig)
    linhas = ''.join(
        '<tr><th>'+escape(r['metrica'])+'</th>'
        f'<td>{r["n_valido"]}/{r["n_total"]}</td>'
        f'<td>{numero(r["media"],4)}</td>'
        f'<td>{numero(r["desvio_padrao_amostral"],4)}</td>'
        f'<td>{numero(r["coeficiente_variacao_pct"],1)}</td>'
        f'<td>{numero(r["minimo"],4)} — {numero(r["maximo"],4)}</td>'
        '<td>'+escape(r['classificacao_descritiva'])+'</td></tr>'
        for r in registros
    )
    figs = ''.join(
        f'<div><h3>{escape(titulo)}</h3><img style="width:100%;max-width:780px" '
        f'src="graficos/consistencia_{registros[i]["chave"]}.png"></div>'
        for i,(titulo,_,__,___) in enumerate(series[:4])
    )
    avisos_html=''.join(f'<li>{escape(t)}</li>' for t in avisos)
    return f'''<section id="consistencia-repeticoes"><h2>Consistência entre repetições</h2>
<p><b>Mesma configuração identificada:</b> {escape(exps[0]['metodo'])}, mapa {escape(str(exps[0]['mapa_id']))}, modalidade {escape(modo)}; {len(exps)} repetições.</p>
<p>CV = 100 × desvio-padrão amostral / |média|. Faixas descritivas propostas: até 10% = baixa dispersão; acima de 10% e até 25% = moderada; acima de 25% = alta. São limites ilustrativos, não padrões científicos universais. Para CV indefinido (média ≈ 0) use desvio absoluto e intervalo.</p>
<p><b>Nota:</b> dispersão baixa não comprova acurácia nem ausência de viés. Com duas execuções, mostramos os valores mas não classificamos a estabilidade. Em <i>rota_fixa</i>, o tempo de conclusão é um resultado experimental, não uma condição de agrupamento.</p>
<div class="scroller"><table><thead><tr><th>Métrica</th><th>N válidos</th><th>Média</th><th>DP</th><th>CV (%)</th><th>Intervalo</th><th>Leitura descritiva</th></tr></thead><tbody>{linhas}</tbody></table></div>
{figs}<p>Arquivos: <code>consistencia_repeticoes.csv</code> e <code>valores_por_repeticao.csv</code>.</p>
{('<ul>'+avisos_html+'</ul>') if avisos_html else ''}</section>'''


# Aliases mais explícitos para testes e integração.
gerar_bloco_consistencia = avaliar
