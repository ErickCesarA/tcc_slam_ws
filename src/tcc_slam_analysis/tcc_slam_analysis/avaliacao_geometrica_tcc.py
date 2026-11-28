#!/usr/bin/env python3
"""Geometria de referência de mundos Gazebo. Módulo para análise ROS 2."""
from __future__ import annotations
import argparse
from bisect import bisect_left
import csv
import json
import math
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np


def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def pose_xml(element):
    if element is None or not (element.text or '').strip():return (0.,0.,0.,0.)
    vals=[float(x) for x in element.text.split()]
    vals += [0.] * (6-len(vals))
    if abs(vals[3])>1e-7 or abs(vals[4])>1e-7:
        raise ValueError('SDF com roll/pitch não nulos: o avaliador trabalha em plano horizontal.')
    if element.get('relative_to'):
        raise ValueError(f"Pose SDF relative_to='{element.get('relative_to')}' exige resolver a árvore de frames.")
    return vals[0],vals[1],vals[2],vals[5]


def comp(a,b):
    x,y,z,t=a; xb,yb,zb,tb=b
    return (x+math.cos(t)*xb-math.sin(t)*yb,
            y+math.sin(t)*xb+math.cos(t)*yb,z+zb,t+tb)


def invert(p):
    x,y,z,t=p
    return (-math.cos(t)*x-math.sin(t)*y,math.sin(t)*x-math.cos(t)*y,-z,-t)


def sdf_models(path,slice_z=.15):
    root=ET.parse(path).getroot()
    world=root.find('world')
    if world is None:raise ValueError('SDF sem elemento <world>')
    caixas=[]; floor=None
    unsupported=[]
    for model in world.findall('model'):
        name=model.get('name','sem_nome')
        if str(model.findtext('static','false')).strip().lower() not in ('true','1'):
            continue
        mp=pose_xml(model.find('pose'))
        for link in model.findall('link'):
            lp=comp(mp,pose_xml(link.find('pose')))
            for collision in link.findall('collision'):
                geo=collision.find('geometry')
                if geo is None:continue
                sz=geo.findtext('./box/size')
                if not sz:
                    unsupported.append(f'{name}/{collision.get("name")}: geometria não-box')
                    continue
                sx,sy,sz_z=map(float,sz.split())
                cp=comp(lp,pose_xml(collision.find('pose')))
                # Parede/obstáculo deve cruzar plano do LiDAR inferior (0.20m).
                if cp[2]-sz_z/2 > slice_z or cp[2]+sz_z/2 < slice_z:
                    # Piso não pode ser tratado como obstáculo.
                    if 'ground' in name.lower() and floor is None:
                        floor=(cp,sx,sy)
                    continue
                # Evita classificar piso elevado como obstáculo se for o chão de área.
                if 'ground' in name.lower() and sz_z < .3:
                    if floor is None:floor=(cp,sx,sy)
                    continue
                caixas.append(dict(nome=name,x=cp[0],y=cp[1],yaw=cp[3],sx=sx,sy=sy,altura=sz_z))
    if world.find('include') is not None:
        unsupported.append('O SDF contém <include> não expandido (geometria omitida).')
    if unsupported:
        raise ValueError("Geometria SDF não totalmente representada (não será feita comparação parcial): " + "; ".join(unsupported))
    if not caixas:raise ValueError(f'Nenhuma caixa de obstáculo encontrada em {path}')
    # ROI baseada no piso; para os mundos deste TCC, 10×10m centrado na origem.
    if floor is None:
        raise ValueError('Piso ground não encontrado: defina manualmente ROI em versão estendida.')
    cp,sx,sy=floor
    if abs(cp[3])>1e-6:
        raise ValueError('Piso rotacionado não suportado como ROI retangular.')
    bounds=(cp[0]-sx/2,cp[1]-sy/2,cp[0]+sx/2,cp[1]+sy/2)
    return world.get('name'),caixas,bounds,unsupported


def identificar_experimento(ws,n):
    root=ws/'bags'
    if not root.is_dir():raise FileNotFoundError(f'Pasta não encontrada: {root}')
    pat=re.compile(rf'^(?:exp_{n}_|experimento_0*{n}$)')
    matches=[p for p in root.iterdir() if p.is_dir() and pat.match(p.name)]
    if len(matches)!=1:
        raise FileNotFoundError(f'Experimento {n}: esperado 1 diretório em bags, encontrados {len(matches)}: {matches}')
    bag=matches[0]/'dados'/'ground_truth'
    if not (bag/'metadata.yaml').exists():
        raise FileNotFoundError(f'Rosbag incompleto/ausente: {bag}')
    match=re.search(r'_map_(\d+)(?:$|_)',matches[0].name)
    if match: mapa=int(match.group(1))
    else:
        manifest=matches[0]/'manifest.json'
        config=json.loads(manifest.read_text()) if manifest.exists() else {}
        mapa=config.get('map_id',config.get('mapa'))
        if mapa is None:raise ValueError(f'Experimento {n}: mapa não registrado; passe --mapa {n}=ID')
        mapa=int(mapa)
    return bag,mapa,matches[0]


def mais_proximo(tempos,poses,t,tolerancia=.5):
    i=bisect_left(tempos,t)
    candidates=[j for j in (i-1,i) if 0<=j<len(tempos)]
    if not candidates:return None
    best=min(candidates,key=lambda j:abs(tempos[j]-t))
    return poses[best] if abs(tempos[best]-t)<=tolerancia else None


def ler_rosbag(bag, duracao_max=300):
    try:
        import rosbag2_py
        from rosidl_runtime_py.utilities import get_message
        from rclpy.serialization import deserialize_message
    except ImportError as exc:
        raise RuntimeError('Execute após source /opt/ros/lyrical/setup.bash e install/setup.bash; rosbag2_py indisponível.') from exc
    # Descobre armazenamento pelo metadata.yaml (MCAP é o formato do TCC).
    storage='mcap'
    meta=(bag/'metadata.yaml').read_text(errors='replace')
    m=re.search(r'^\s*storage_identifier:\s*([\w-]+)',meta,re.M)
    if m:storage=m.group(1)
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id=storage),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr',output_serialization_format='cdr'))
    tipos={t.name:t.type for t in reader.get_all_topics_and_types()}
    desejados={'/map','/tf','/ground_truth/odom','/exploration/events'}
    classes={t:get_message(tipos[t]) for t in desejados if t in tipos}
    if '/map' not in classes:raise RuntimeError(f'Rosbag {bag} não contém /map')
    mapas=[];gt=[];map_odom=[];odom_base=[];first_goal=None
    ultima_ate_fim=None
    while reader.has_next():
        topico,raw,_=reader.read_next()
        if topico not in classes:continue
        msg=deserialize_message(raw,classes[topico])
        if topico=='/map':
            t=float(msg.header.stamp.sec)+msg.header.stamp.nanosec*1e-9
            if t<=0:continue
            mapas.append((t,{
                'data':np.asarray(msg.data,dtype=np.int16).reshape((msg.info.height,msg.info.width)).copy(),
                'resolution':float(msg.info.resolution),
                'origin':(float(msg.info.origin.position.x),float(msg.info.origin.position.y),0.,yaw(msg.info.origin.orientation)),
                'frame':str(msg.header.frame_id).lstrip('/'),
                'time':t,
            }))
            # evita reter muitos mapas: mantém último; se t0 ainda desconhecido, a gravação já é temporal.
            if len(mapas)>3:mapas.pop(0)
            if first_goal is not None and t<=first_goal+duracao_max+2:
                ultima_ate_fim=mapas[-1][1]
        elif topico=='/ground_truth/odom':
            t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if t>0:gt.append((t,(float(msg.pose.pose.position.x),float(msg.pose.pose.position.y),0.,yaw(msg.pose.pose.orientation))))
        elif topico=='/tf':
            for tr in msg.transforms:
                t=tr.header.stamp.sec+tr.header.stamp.nanosec*1e-9
                if t<=0:continue
                parent=tr.header.frame_id.lstrip('/');child=tr.child_frame_id.lstrip('/')
                p=(float(tr.transform.translation.x),float(tr.transform.translation.y),0.,yaw(tr.transform.rotation))
                if parent=='map' and child=='odom':map_odom.append((t,p))
                if parent=='odom' and child=='base_link':odom_base.append((t,p))
                if parent=='map' and child=='base_link':map_odom.append((t,(p,'direct')))
        elif topico=='/exploration/events':
            try:
                e=json.loads(msg.data)
                if e.get('event')=='GOAL_SENT' and first_goal is None:
                    first_goal=float(e.get('timestamp'))
                    anteriores=[v for t,v in mapas if t<=first_goal+duracao_max+2]
                    if anteriores:ultima_ate_fim=anteriores[-1]
            except (ValueError,TypeError):pass
    if not mapas:raise RuntimeError(f'Rosbag {bag}: não há mensagens /map com timestamp válido')
    if first_goal is not None and ultima_ate_fim is not None:
        grade=ultima_ate_fim
    else:
        grade=mapas[-1][1]
        if first_goal is not None:
            print('AVISO: não foi encontrada grade dentro da janela experimental; usando última /map da bag.',file=sys.stderr)
        else:
            print('AVISO: ausência de GOAL_SENT; não foi possível determinar a janela experimental.',file=sys.stderr)
    return grade,gt,map_odom,odom_base,first_goal


def calcular_alinhamento(gt,mo,ob,t_inicio=None):
    if not gt or not mo or not ob:
        raise RuntimeError('Faltam poses /ground_truth/odom ou TF map->odom / odom->base_link para alinhar o mapa.')
    gt.sort();mo.sort();ob.sort()
    tmo=[x[0] for x in mo];tob=[x[0] for x in ob]
    pmo=[x[1] for x in mo];pob=[x[1] for x in ob]
    for t,p_ref in gt:
        if t_inicio is not None and t<t_inicio:continue
        p1=mais_proximo(tmo,pmo,t)
        if p1 is None:continue
        if isinstance(p1[1],str) and p1[1]=='direct':
            p_est=p1[0]
        else:
            p2=mais_proximo(tob,pob,t)
            if p2 is None:continue
            p_est=comp(p1,p2)
        # GroundTruth no referencial Gazebo world, SLAM no referencial map.
        transform=comp(p_ref,invert(p_est))
        return transform,t
    raise RuntimeError('Não há pares TF/GT temporariamente compatíveis após GOAL_SENT.')


def world_coordinates(grade,world_map):
    data=grade['data']
    res=grade['resolution']
    if res<=0 or data.size==0:raise ValueError('OccupancyGrid vazio ou resolução inválida')
    h,w=data.shape
    iy,ix=np.indices((h,w),dtype=np.float32)
    xlocal=(ix+.5)*res;ylocal=(iy+.5)*res
    pose=comp(world_map,grade['origin'])
    x=pose[0]+math.cos(pose[3])*xlocal-math.sin(pose[3])*ylocal
    y=pose[1]+math.sin(pose[3])*xlocal+math.cos(pose[3])*ylocal
    return x,y


def reamostrar_para_mundo(grade,world_map,bounds,res_out=.05):
    """Amostra a grade SLAM numa grade fixa do mundo Gazebo (comparabilidade entre resoluções)."""
    if res_out<=0:raise ValueError('Resolução de avaliação deve ser positiva.')
    xmin,ymin,xmax,ymax=bounds
    w=int(math.ceil((xmax-xmin)/res_out));h=int(math.ceil((ymax-ymin)/res_out))
    yy,xx=np.indices((h,w),dtype=np.float32)
    wx=xmin+(xx+.5)*res_out
    wy=ymin+(yy+.5)*res_out
    wp=comp(world_map,grade['origin'])
    dx=wx-wp[0];dy=wy-wp[1]
    c=math.cos(wp[3]);s=math.sin(wp[3])
    ux=c*dx+s*dy
    uy=-s*dx+c*dy
    ix=np.floor(ux/grade['resolution']).astype(np.int64)
    iy=np.floor(uy/grade['resolution']).astype(np.int64)
    valid=(ix>=0)&(iy>=0)&(ix<grade['data'].shape[1])&(iy<grade['data'].shape[0])
    data=np.full((h,w),-1,dtype=np.int16)
    data[valid]=grade['data'][iy[valid],ix[valid]]
    return {'data':data,'resolution':res_out,'origin':(xmin,ymin,0.,0.),
            'frame':'world','time':grade['time']}


def dentro_caixa(x,y,b):
    dx=x-b['x'];dy=y-b['y'];c=math.cos(b['yaw']);s=math.sin(b['yaw'])
    u=c*dx+s*dy;v=-s*dx+c*dy
    return (np.abs(u)<=b['sx']/2) & (np.abs(v)<=b['sy']/2)


def accuracy(pred,gt,known,roi,cellarea):
    relevant=roi & known
    tp=int(np.count_nonzero(relevant & pred & gt))
    fp=int(np.count_nonzero(relevant & pred & ~gt))
    fn=int(np.count_nonzero(relevant & ~pred & gt))
    tn=int(np.count_nonzero(relevant & ~pred & ~gt))
    ignor=int(np.count_nonzero(roi & ~known & gt))
    ignor_livre=int(np.count_nonzero(roi & ~known & ~gt))
    def div(a,b):return round(a/b,6) if b else None
    area_ref=int(np.count_nonzero(roi & gt))*cellarea
    return {
       'celulas_ROI':int(np.count_nonzero(roi)),
       'cobertura_conhecida_pct':100*div(int(np.count_nonzero(relevant)),int(np.count_nonzero(roi))) if np.count_nonzero(roi) else None,
       'area_obstaculos_gazebo_m2':area_ref,
       'area_livre_gazebo_m2':int(np.count_nonzero(roi & ~gt))*cellarea,
       'cobertura_livre_real_pct':100*div(tn+fp,tn+fp+ignor_livre) if tn+fp+ignor_livre else None,
       'classificacao_livre_correta_global_pct':100*div(tn,tn+fp+ignor_livre) if tn+fp+ignor_livre else None,
       'area_obstaculos_estimados_m2':int(np.count_nonzero(relevant & pred))*cellarea,
       'area_intersecao_obstaculos_m2':tp*cellarea,
       'area_falsos_ocupados_m2':fp*cellarea,
       'area_obstaculos_livres_por_erro_m2':fn*cellarea,
       'area_obstaculos_desconhecidos_m2':ignor*cellarea,
       'TP':tp,'FP':fp,'FN_conhecido':fn,'TN':tn,
       'precisao_obstaculos_conhecido':div(tp,tp+fp),
       'recall_obstaculos_conhecido':div(tp,tp+fn),
       'recall_obstaculos_global':div(tp,tp+fn+ignor),
       'IoU_obstaculos_conhecido':div(tp,tp+fp+fn),
       'IoU_obstaculos_global':div(tp,tp+fp+fn+ignor),
       'F1_obstaculos_conhecido':div(2*tp,2*tp+fp+fn),
       'acuracia_celulas_conhecidas':div(tp+tn,tp+fp+fn+tn),
    }


def localizar_bordas(pred,gt,known,roi,x,y):
    # Distância entre bordas ocupadas estimadas e bordas físicas reais.
    try:
        from scipy.ndimage import binary_erosion
        from scipy.spatial import cKDTree
    except ImportError:
        return {'erro_bordas_medio_m':None,'erro_bordas_p95_m':None,'cobertura_bordas_ref_ate_0_15m':None}
    pe=(pred & known & roi) & ~binary_erosion(pred & known & roi)
    ge=(gt & roi) & ~binary_erosion(gt & roi)
    if not pe.any() or not ge.any():
        return {'erro_bordas_medio_m':None,'erro_bordas_p95_m':None,'cobertura_bordas_ref_ate_0_15m':None}
    a=np.column_stack((x[pe],y[pe]));b=np.column_stack((x[ge],y[ge]))
    d,_=cKDTree(b).query(a,k=1)
    dr,_=cKDTree(a).query(b,k=1)
    return {'erro_bordas_medio_m':float(d.mean()),
            'erro_bordas_p95_m':float(np.percentile(d,95)),
            'cobertura_bordas_ref_ate_0_15m':float(np.mean(dr<=.15))}


def plotar(saida, x,y,grade,gt,roi,caixas,bounds,world_name,metrics):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(8.5,8))
    # Posicionamento exato de OccupancyGrid: plot geométrico, não leitura de PNG.
    pred=(grade['data']>=metrics['_threshold'])
    known=(grade['data']>=0)
    ax.scatter(x[roi & known & ~pred],y[roi & known & ~pred],s=1.0,c='#c2ddeb',rasterized=True,label='SLAM: livre')
    ax.scatter(x[roi & ~known],y[roi & ~known],s=1.0,c='#d8d8d8',rasterized=True,label='SLAM: desconhecido')
    ax.scatter(x[roi & pred],y[roi & pred],s=3.0,c='#dc554d',rasterized=True,label='SLAM: ocupado')
    from matplotlib.patches import Polygon
    legend_drawn=False
    for b in caixas:
        hx=b['sx']/2;hy=b['sy']/2;c=math.cos(b['yaw']);s=math.sin(b['yaw'])
        poly=[]
        for u,v in [(-hx,-hy),(hx,-hy),(hx,hy),(-hx,hy)]:
            poly.append((b['x']+c*u-s*v,b['y']+s*u+c*v))
        ax.add_patch(Polygon(poly,closed=True,fill=False,edgecolor='#148c46',linewidth=1.8,label='Gazebo: colisão' if not legend_drawn else None))
        legend_drawn=True
    ax.set_xlim(bounds[0]-.3,bounds[2]+.3);ax.set_ylim(bounds[1]-.3,bounds[3]+.3)
    ax.set_aspect('equal');ax.grid(alpha=.2)
    ax.set_xlabel('X mundo Gazebo (m)');ax.set_ylabel('Y mundo Gazebo (m)')
    ax.set_title(f'Grade SLAM × obstáculos reais | {world_name}\n'
                 f"IoU conhecido={metrics.get('IoU_obstaculos_conhecido')}  | cobertura={metrics.get('cobertura_conhecida_pct') or 0:.1f}%")
    ax.legend(loc='upper right',markerscale=2,fontsize=8)
    fig.tight_layout();fig.savefig(saida,dpi=175);plt.close(fig)


def avaliar(grade,world_map,world_file,output,threshold=50,slice_z=.15,alinhamento_origem='inicial',t_calib=None,meta=None,res_avaliacao=.05):
    world_name,caixas,bounds,unsupported=sdf_models(world_file,slice_z)
    output.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(output/'mapa_final_bruto.npz',
                        data=grade['data'],resolution=grade['resolution'],
                        origin=np.asarray(grade['origin']),time=grade['time'],
                        transform_world_from_map=np.asarray(world_map))
    # Sempre avalia no mesmo grid do mundo, mesmo quando o SLAM publica em 0.05/0.10m.
    resolucao_nativa=grade['resolution']
    frame_nativo=grade['frame']
    rx,ry=world_coordinates(grade,world_map)
    fora=(rx<bounds[0])|(rx>bounds[2])|(ry<bounds[1])|(ry>bounds[3])
    ocupada_fora=int(np.count_nonzero(fora & (grade['data']>=threshold)))*(resolucao_nativa**2)
    conhecida_fora=int(np.count_nonzero(fora & (grade['data']>=0)))*(resolucao_nativa**2)
    grade=reamostrar_para_mundo(grade,world_map,bounds,res_avaliacao)
    x,y=world_coordinates(grade,(0.,0.,0.,0.))
    roi=(x>=bounds[0])&(x<=bounds[2])&(y>=bounds[1])&(y<=bounds[3])
    if np.count_nonzero(roi)<100:
        raise RuntimeError('Sobreposição com o piso Gazebo é muito pequena. Verifique alinhamento SE(2) ou SDF selecionado.')
    gt=np.zeros(grade['data'].shape,dtype=bool)
    for b in caixas:gt|=dentro_caixa(x,y,b)
    pred=grade['data']>=threshold
    known=grade['data']>=0
    c=grade['resolution']**2
    data=accuracy(pred,gt,known,roi,c)
    data.update(localizar_bordas(pred,gt,known,roi,x,y))
    data.update({'mundo':world_name,'sdf':str(world_file),
                 'num_colisoes_obstaculos':len(caixas),
                 'num_objetos_distintos':len(set(b['nome'] for b in caixas)),
                 'map_frame_original':frame_nativo,'map_resolution_native_m':resolucao_nativa,
                 'evaluation_resolution_m':res_avaliacao,
                 'area_ocupada_fora_piso_nominal_m2':ocupada_fora,
                 'area_conhecida_fora_piso_nominal_m2':conhecida_fora,
                 'map_time_sim_s':grade['time'],'limiar_ocupado':threshold,
                 'plano_z_m':slice_z,
                 'alinhamento':alinhamento_origem,'transform_world_from_map':list(world_map),
                 'tempo_calibracao_sim_s':t_calib,'ROI_world_m':bounds,
                 'limitacoes_geometria':unsupported})
    if meta:data.update(meta)
    objetos=[]
    for b in caixas:
        p=dentro_caixa(x,y,b)&roi
        known_n=int(np.count_nonzero(p&known))
        tp=int(np.count_nonzero(p&known&pred))
        # Centroide de células ocupadas na vizinhança: diagnóstico aproximado,
        # não representa necessariamente o centro do objeto se visto parcialmente.
        probed=(roi & known & pred & dentro_caixa(x,y,dict(b,sx=b['sx']+.30,sy=b['sy']+.30)))
        # Não reportar centroide para paredes de perímetro (grande truncamento).
        if np.count_nonzero(probed)>=5 and not b['nome'].lower().startswith('wall_'):
            cx=float(np.mean(x[probed]));cy=float(np.mean(y[probed]))
            dc=float(math.hypot(cx-b['x'],cy-b['y']))
        else:cx=cy=dc=None
        objetos.append({'obstaculo':b['nome'],'x_real_m':b['x'],'y_real_m':b['y'],
                        'centroide_ocupacao_vizinha_x_m':cx,'centroide_ocupacao_vizinha_y_m':cy,
                        'distancia_centroide_observado_ao_centro_real_m':dc,
                        'area_geom_m2':b['sx']*b['sy'],
                        'area_estimativa_real_m2':np.count_nonzero(p)*c,
                        'area_mapeada_como_ocupada_m2':tp*c,
                        'area_classificada_livre_erroneamente_m2':np.count_nonzero(p&known&~pred)*c,
                        'area_desconhecida_m2':np.count_nonzero(p&~known)*c,
                        'recall_obstaculo_conhecido':tp/known_n if known_n else None})
    output.mkdir(parents=True,exist_ok=True)
    (output/'metricas_geometricas.json').write_text(json.dumps(data,ensure_ascii=False,indent=2,default=float))
    with (output/'obstaculos_por_objeto.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=list(objetos[0]))
        writer.writeheader();writer.writerows(objetos)
    plotdata=dict(data,_threshold=threshold)
    plotar(output/'sobreposicao_mapa_gazebo.png',x,y,grade,gt,roi,caixas,bounds,world_name,plotdata)
    return data

