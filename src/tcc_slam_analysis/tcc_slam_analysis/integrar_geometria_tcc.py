#!/usr/bin/env python3
"""Integra avaliação geométrica no HTML/CSV do analisador do TCC.

Usa os dados que o analisador já leu do rosbag: não cria nova leitura, não
modifica SLAM/Nav2/experimentos. Falhas geométricas são registradas como N/D.
"""
from __future__ import annotations

from html import escape
from pathlib import Path
import hashlib
import json
import math
import re
import shutil
import sys

import numpy as np
import pandas as pd

from .avaliacao_geometrica_tcc import avaliar, calcular_alinhamento

CHAVES = {
    'geometria_iou_conhecido': 'IoU_obstaculos_conhecido',
    'geometria_iou_global': 'IoU_obstaculos_global',
    'geometria_precisao_obstaculos': 'precisao_obstaculos_conhecido',
    'geometria_recall_obstaculos_conhecidos': 'recall_obstaculos_conhecido',
    'geometria_recall_obstaculos_global': 'recall_obstaculos_global',
    'geometria_f1_conhecido': 'F1_obstaculos_conhecido',
    'geometria_cobertura_conhecida_pct': 'cobertura_conhecida_pct',
    'geometria_cobertura_livre_real_pct': 'cobertura_livre_real_pct',
    'geometria_area_falsos_ocupados_m2': 'area_falsos_ocupados_m2',
    'geometria_area_obstaculos_desconhecidos_m2': 'area_obstaculos_desconhecidos_m2',
    'geometria_area_obstaculos_livres_por_erro_m2': 'area_obstaculos_livres_por_erro_m2',
    'geometria_erro_bordas_medio_m': 'erro_bordas_medio_m',
    'geometria_erro_bordas_p95_m': 'erro_bordas_p95_m',
    'geometria_cobertura_bordas_015m': 'cobertura_bordas_ref_ate_0_15m',
    'geometria_area_ocupada_fora_piso_m2': 'area_ocupada_fora_piso_nominal_m2',
    'geometria_resolucao_mapa_m': 'map_resolution_native_m',
}


def extrair_mapa_id(experimento: Path, manifest: dict) -> int:
    dado = manifest.get('map_id', manifest.get('mapa'))
    if dado is None:
        match = re.search(r'_map_(\d+)(?:_|$)', experimento.name)
        dado = match.group(1) if match else None
    if dado is None:
        raise ValueError('Mapa não identificado no manifest nem no nome da pasta; geometria não calculada.')
    return int(dado)


def localizar_sdf(bag: Path):
    exp = bag.parent.parent
    # bag = .../bags/<experimento>/dados/ground_truth
    if exp.parent.name != 'bags':
        raise ValueError(f'Rosbag fora de bags/<experimento>/dados/ground_truth: {bag}')
    manifest_path = exp / 'manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8')) if manifest_path.exists() else {}
    mapa = extrair_mapa_id(exp, manifest)
    snap = exp / 'mundo_utilizado.sdf'
    if snap.is_file():
        return snap, mapa, manifest, 'snapshot_gravacao', exp
    nome = 'tcc_world.sdf' if mapa == 1 else f'tcc_world_{mapa}.sdf'
    ws = exp.parent.parent
    installed = ws / 'install/tcc_slam_description/share/tcc_slam_description/worlds' / nome
    source = ws / 'src/tcc_slam_description/worlds' / nome
    file = installed if installed.exists() else source
    if not file.is_file():
        raise FileNotFoundError(f'SDF de referência para mapa {mapa} não encontrado: {file}')
    return file, mapa, manifest, 'sdf_atual_nao_verificado', exp


def alinhar_usando_dados(dados, inicio):
    ref = dados['referencia']
    tf = dados['tf']
    if ref.empty or tf.empty:
        raise ValueError('Ground Truth ou TF ausentes; não é possível alinhar o mapa ao mundo.')
    # Os três métodos publicam map->odom e a simulação odom->base_link.
    def serie(parent, child):
        sel = tf[(tf['pai'] == parent) & (tf['filho'] == child)]
        return [(float(r.tempo_s), (float(r.x), float(r.y), 0., float(r.theta)))
                for r in sel.itertuples(index=False)]
    gt = [(float(r.tempo_s), (float(r.x), float(r.y), 0., float(r.theta)))
          for r in ref.itertuples(index=False)]
    mo = serie('map','odom')
    ob = serie('odom','base_link')
    if not mo and not ob:
        raise ValueError('TF map/odom não encontrado no rosbag.')
    if not mo:
        # Um método pode fornecer map->base_link diretamente.
        direct = serie('map','base_link')
        if direct:
            mo = [(t, (p,'direct')) for t,p in direct]
    if not mo:
        raise ValueError('Falta map->odom ou map->base_link no TF.')
    return calcular_alinhamento(gt, mo, ob, inicio)


def gravar_metricas(csv_path, met):
    if not csv_path.is_file():
        raise FileNotFoundError(f'Métricas principais ausentes: {csv_path}')
    df = pd.read_csv(csv_path)
    if len(df) != 1:
        raise ValueError('metricas.csv deve ter exatamente uma linha.')
    for key, value in met.items():
        df.loc[0, key] = value
    df.to_csv(csv_path, index=False)


def valor(v, frac=False, casas=3):
    try:
        if v is None or not math.isfinite(float(v)):
            return 'N/D'
        return f'{float(v) * (100 if frac else 1):.{casas}f}'
    except (TypeError, ValueError):
        return 'N/D'


def montar_html(met, erro=None):
    if erro:
        return ('<section id="avaliacao-geometrica"><h2>Fidelidade geométrica do mapa</h2>'
                '<p>Não foi possível calcular a avaliação geométrica: '
                f'<strong>{escape(str(erro))}</strong>.</p>'
                '<p>Os indicadores anteriores continuam válidos; ausência de resultado geométrico não equivale a nota zero.</p></section>')
    linhas = [
        ('IoU de obstáculos — células conhecidas', valor(met.get('IoU_obstaculos_conhecido'))),
        ('IoU global — desconhecidos penalizados', valor(met.get('IoU_obstaculos_global'))),
        ('Precisão de obstáculos', valor(met.get('precisao_obstaculos_conhecido'))),
        ('Recall de obstáculos conhecidos', valor(met.get('recall_obstaculos_conhecido'))),
        ('Cobertura da região de referência', valor(met.get('cobertura_conhecida_pct')) + '%'),
        ('Erro médio das bordas', valor(met.get('erro_bordas_medio_m'))+' m'),
        ('Erro P95 das bordas', valor(met.get('erro_bordas_p95_m'))+' m'),
        ('Falsos obstáculos', valor(met.get('area_falsos_ocupados_m2'), casas=2)+' m²'),
        ('Obstáculos ainda desconhecidos', valor(met.get('area_obstaculos_desconhecidos_m2'), casas=2)+' m²'),
        ('Obstáculos classificados incorretamente como livres', valor(met.get('area_obstaculos_livres_por_erro_m2'), casas=2)+' m²'),
    ]
    trs=''.join(f'<tr><th style="text-align:left;padding:9px;border-bottom:1px solid #ddd">{escape(k)}</th>'
                f'<td style="text-align:right;padding:9px;border-bottom:1px solid #ddd">{escape(v)}</td></tr>'
                for k,v in linhas)
    detalhe_objetos = ''
    info_objetos = met.get('_objetos_html') or ''
    if info_objetos:
        detalhe_objetos = ('<h3>Obstáculos individuais (diagnóstico aproximado)</h3>'
                           + '<p>Centroides são estimados a partir de células ocupadas próximas ao objeto; '
                             'não são uma medição independente do seu centro geométrico.</p>'
                           + info_objetos)
    source = ('SDF preservado no momento do experimento' if met.get('geometria_fonte_sdf') == 'snapshot_gravacao'
              else 'ATENÇÃO: SDF atual do workspace; não há confirmação de que era idêntico ao usado na execução')
    return f'''<section id="avaliacao-geometrica"><h2>Fidelidade geométrica: mapa estimado × Gazebo</h2>
    <p>Referência: <code>{escape(str(met.get('sdf','')))}</code>. {escape(source)}.</p>
    <p>Uma grade de 0,05 m/célula é utilizada para avaliar a região do piso do mundo; o alinhamento entre referenciais usa
    a primeira correspondência temporal disponível entre TF e Ground Truth no intervalo.</p>
    <img src="geometria/sobreposicao_mapa_gazebo.png" alt="Obstáculos reais SDF sobrepostos à grade estimada"
         style="max-width:100%;height:auto"/>
    <table style="width:100%;border-collapse:collapse"><tbody>{trs}</tbody></table>
    <p><b>Interpretação:</b> IoU/precisão/recall entre 0 e 1 (maior é melhor); erro de borda em metros (menor é melhor).
    <b>Não conhecido ≠ classificado livre.</b> O LiDAR observa superfícies, não necessariamente o interior de objetos opacos.
    A IoU sobre caixas sólidas deve ser interpretada junto da cobertura e do erro de borda.</p>
    {detalhe_objetos}
    <p>Arquivos: <code>geometria/metricas_geometricas.json</code>, <code>geometria/obstaculos_por_objeto.csv</code>
    e <code>geometria/sobreposicao_mapa_gazebo.png</code>.</p></section>'''


def inserir_secao_html(path: Path, secao):
    if not path.is_file():
        raise FileNotFoundError(f'Relatório individual HTML ausente: {path}')
    html = path.read_text(encoding='utf-8')
    import re as _re
    html = _re.sub(r'<section id="avaliacao-geometrica">.*?</section>', '', html, flags=_re.S)
    if '</main>' not in html:
        raise ValueError('HTML sem </main>; não foi alterado.')
    html = html.replace('</main>', secao+'\n</main>', 1)
    path.write_text(html, encoding='utf-8')


def integrar(bag, resultados, dados, inicio, fim):
    """Pós-processamento da análise individual. Nunca inventa zeros quando falta evidência."""
    bag, resultados = Path(bag).resolve(), Path(resultados).resolve()
    csv = resultados/'metricas.csv'
    html = resultados/'relatorio.html'
    try:
        sdf, mapa_id, manifest, origem, _ = localizar_sdf(bag)
        # Seleciona /map dentro da janela efetiva (não o último pacote pós-término).
        candidatos = [p for p in dados.get('mapas_recentes', [])
                      if inicio-2. <= float(p['tempo_s']) <= fim+2.]
        grade = max(candidatos, key=lambda p: float(p['tempo_s'])) if candidatos else dados['mapa_final']
        if grade is None:
            raise ValueError('Nenhuma grade /map presente no rosbag.')
        if str(grade['frame_id']).strip('/') != 'map':
            raise ValueError('Frame de /map diferente de map; não será feita comparação com TF incorreto.')
        if not (inicio - 2. <= float(grade['tempo_s']) <= fim + 2.):
            raise ValueError('Última grade /map fora do intervalo experimental (margem ±2s).')
        transform, t = alinhar_usando_dados(dados, inicio)
        g={'data':grade['data'], 'resolution':float(grade['resolucao']),
           'origin':(float(grade['origem_x']),float(grade['origem_y']),0.,float(grade.get('origem_theta',0.))),
           'frame':'map', 'time':float(grade['tempo_s'])}
        hash_sdf = hashlib.sha256(sdf.read_bytes()).hexdigest()
        proveniencia = manifest.get('hashes_configuracao_sha256', {})
        original_hash = manifest.get('mundo_sdf_sha256') or (
            proveniencia.get('mundo_sdf') if isinstance(proveniencia,dict) else None)
        if original_hash and original_hash != hash_sdf:
            raise ValueError('SDF de referência não corresponde ao SHA-256 registrado no experimento.')
        data=avaliar(g, transform, sdf, resultados/'geometria',
                     threshold=50, slice_z=.15, alinhamento_origem='primeiro_par_GT_TF',
                     t_calib=t, res_avaliacao=.05,
                     meta={'experimento_mapa':mapa_id,
                           'geometria_fonte_sdf':origem,
                           'geometria_sha256_sdf':hash_sdf,
                           'geometria_intervalo_fim_s':fim})
        # Comparação por objeto: área em m² e posição do centróide observado.
        por_objeto = pd.read_csv(resultados/'geometria'/'obstaculos_por_objeto.csv')
        out_obj = []
        for r in por_objeto.itertuples(index=False):
            nome = str(getattr(r,'obstaculo', ''))
            if nome.lower().startswith('wall_'):
                continue  # Paredes de contorno não têm centroide observável confiável.
            out_obj.append((nome, getattr(r,'area_geom_m2',None),
                            getattr(r,'area_mapeada_como_ocupada_m2',None),
                            getattr(r,'distancia_centroide_observado_ao_centro_real_m',None)))
        linhas_obj = []
        for nome,area_real,area_mapeada,erro_pos in out_obj[:20]:
            linhas_obj.append('<tr><th style="text-align:left">'+escape(nome)+'</th><td>'
                  +escape(valor(area_real,casas=2))+'</td><td>'
                  +escape(valor(area_mapeada,casas=2))+'</td><td>'
                  +escape(valor(erro_pos,casas=3))+'</td></tr>')
        data['_objetos_html'] = ('<table style="width:100%;border-collapse:collapse"><thead>'
                                '<tr><th>Objeto</th><th>Área real (m²)</th><th>Área observada ocupada (m²)</th>'
                                '<th>Distância entre centroides (m)</th></tr></thead><tbody>'
                                + ''.join(linhas_obj)+'</tbody></table>')
        erros_obj=[float(q[3]) for q in out_obj if q[3] is not None and math.isfinite(float(q[3]))]
        data['geometria_erro_centroide_medio_m'] = float(np.mean(erros_obj)) if erros_obj else None
        gravar_metricas(csv, {**{k:data.get(v) for k,v in CHAVES.items()},
                               'geometria_erro_centroide_medio_m':data['geometria_erro_centroide_medio_m'],
                               'geometria_status':'ok',
                               'geometria_fonte_sdf':origem,
                               'geometria_sha256_sdf':hash_sdf,
                               'geometria_mapa_id':mapa_id,
                               'geometria_resolucao_avaliacao_m':.05})
        inserir_secao_html(html, montar_html(data))
        print('Avaliação geométrica: OK | IoU conhecido =', data.get('IoU_obstaculos_conhecido'))
        return True
    except (OSError, ValueError, RuntimeError, ImportError, IndexError, KeyError, TypeError) as exc:
        aviso=f'{type(exc).__name__}: {exc}'
        print('AVISO — geometria não calculada:', aviso, file=sys.stderr)
        # Os resultados normais do experimento NÃO são invalidados por falha extra.
        try:
            gravar_metricas(csv, {'geometria_status':'indisponivel', 'geometria_erro':aviso})
            inserir_secao_html(html, montar_html(None, erro=aviso))
        except Exception as e:
            print('AVISO — também não foi possível inserir nota geométrica:', e, file=sys.stderr)
        return False
