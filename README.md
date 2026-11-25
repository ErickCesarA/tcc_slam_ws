# Comparação de Métodos SLAM em Robô Móvel Autônomo

**ROS 2 Lyrical · Gazebo Sim · Nav2 · MRPT · SLAM Toolbox**

Projeto de Trabalho de Conclusão de Curso (TCC) em **Engenharia Mecânica — Instituto Federal da Paraíba (IFPB)**. O sistema simula um robô móvel de tração diferencial e permite estudar diferentes métodos de **Localização e Mapeamento Simultâneos (SLAM)** em ambientes virtuais, com navegação autônoma, aquisição de dados e avaliação quantitativa.

> **Escopo científico:** a exploração autônoma avalia o desempenho do **sistema integrado SLAM + navegação + exploração**. Os experimentos com trajetória fixa e degradação de sensores estão previstos como extensões e ainda não fazem parte do fluxo principal documentado aqui.

## Funcionalidades

- Simulação de um **robô diferencial**, com LiDAR 2D, IMU, odometria e descrição em Xacro/URDF.
- Dois cenários SDF selecionáveis pelo parâmetro `map`.
- Escolha de **quatro métodos de SLAM** pelo parâmetro `slam_method`.
- Navegação com **Nav2** e exploração autônoma por fronteiras (*Frontier Explorer V5*).
- Registro de posições de referência (*ground truth*) fornecidas pelo Gazebo.
- Gravação experimental com **rosbag2**, controle por tempo simulado e eventos de exploração.
- Cálculo de métricas de trajetória, mapeamento e navegação.
- Relatórios individuais e comparativos em **HTML**, com gráficos, CSVs e imagem do mapa final.

## Métodos implementados

| Identificador | Implementação | Abordagem |
| --- | --- | --- |
| `toolbox` | SLAM Toolbox | SLAM 2D baseado em grafos |
| `graph` | MRPT GraphSLAM | SLAM baseado em grafos |
| `rbpf` | MRPT RBPF-SLAM | Filtro de partículas Rao-Blackwellizado |
| `ekf` | MRPT EKF-SLAM 2D | Filtro de Kalman Estendido e landmarks |

Os pacotes MRPT foram adaptados para a pilha ROS 2 do projeto, incluindo interfaces de mensagens, sincronização temporal e transformações TF. A implementação EKF utiliza dois componentes auxiliares: um detector de *landmarks* a partir do LiDAR e um construtor de grade de ocupação para disponibilizar `/map` ao Nav2.

> As adaptações de integração não devem ser confundidas com uma implementação própria das formulações matemáticas fundamentais dos algoritmos MRPT.

## Arquitetura

```text
                  Gazebo Sim
            mundo + robô + sensores
                       │
                 ros_gz_bridge
                       │
          /scan  /odom  /imu/data  /clock
                       │
                       ▼
   ┌──────────────────────────────────────┐
   │ Método selecionado:                 │
   │ Toolbox · MRPT Graph · RBPF · EKF   │
   └──────────────────────────────────────┘
                │               │
              /map          map → odom
                └──────┬────────┘
                       ▼
              Frontier Explorer V5
                       │
                       ▼
               Nav2 /navigate_to_pose
                       │
                     /cmd_vel
                       │
                       ▼
                   Gazebo Sim

    /ground_truth/odom + /tf + /map + eventos
                       │
                       ▼
                rosbag2 → análise
                       │
                 CSV + PNG + HTML
```

A árvore de referenciais utilizada na navegação segue a composição `map → odom → base_link → lidar_link` (com `imu_link` vinculado ao robô). Somente o método SLAM selecionado deve publicar a correção global `map → odom`.

## Organização do repositório

```text
tcc_slam_ws/
├── src/
│   ├── tcc_slam_description/     # URDF/Xacro e mundos SDF
│   ├── tcc_slam_bringup/         # launchers, configuração Nav2 e MRPT
│   ├── tcc_slam_ground_truth/    # referência de pose da simulação
│   ├── tcc_slam_exploration/     # Frontier Explorer V5
│   ├── tcc_slam_analysis/        # processamento e relatórios
│   ├── mrpt_ekf_slam_2d/        # integração MRPT EKF
│   ├── mrpt_rbpf_slam/          # integração MRPT RBPF
│   └── mrpt_graphslam_2d/       # integração MRPT GraphSLAM
├── executar_experimento.py      # aquisição e controle de experimentos
├── analisar_experimento.sh      # reanálise por número
├── comparar_experimento.py      # comparação entre experimentos
├── bags/                        # gravações rosbag2 (normalmente locais)
├── results/                     # relatórios e métricas produzidos
├── third_party/                 # atribuições/licenças de terceiros
└── README.md
```

`build/`, `install/` e `log/` são gerados pelo `colcon` e não devem ser versionados. As gravações em `bags/` podem ser grandes; mantenha-as fora do Git e disponibilize conjuntos de dados separadamente, quando necessário.

## Ambiente de desenvolvimento

Configuração utilizada no desenvolvimento:

- **Ubuntu 26.04** e **ROS 2 Lyrical**;
- **Gazebo Sim** e `ros_gz_bridge`;
- **Nav2**, **RViz2** e **SLAM Toolbox**;
- **MRPT 3.3.1** e pacotes ROS de integração MRPT;
- **Python 3**, NumPy, Pandas e Matplotlib;
- **C++**, `colcon`, `ament`, `rosbag2` e `tf2`.

O projeto possui adaptações específicas para essa combinação. Não se deve presumir compatibilidade imediata com outras distribuições do ROS 2 ou versões do MRPT sem recompilar e validar os pacotes.

### Preparar o workspace

Com ROS 2 Lyrical e as dependências acima instaladas:

```bash
source /opt/ros/lyrical/setup.bash
cd ~/tcc_slam_ws

# Opcional, quando rosdep estiver configurado e as dependências
# estiverem disponíveis nos repositórios da distribuição:
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=OFF

source install/setup.bash
```

> Alguns componentes MRPT exigiram adaptações de compatibilidade, presentes neste repositório. Se houver erro de dependências, consulte os `package.xml` e os arquivos de configuração de cada pacote; `rosdep` pode não resolver todas as dependências específicas automaticamente.

## Executar simulação, SLAM e Nav2

O arquivo `navigation.launch.py` inicializa a simulação compartilhada, o método SLAM escolhido, os componentes auxiliares e a infraestrutura Nav2. **Não abra outra instância do `simulation.launch.py` em paralelo.**

```bash
source /opt/ros/lyrical/setup.bash
source ~/tcc_slam_ws/install/setup.bash

# Exemplo: mundo 1 com SLAM Toolbox
ros2 launch tcc_slam_bringup navigation.launch.py \
  map:=1 slam_method:=toolbox
```

Outras combinações:

```bash
# Mundo 2 com MRPT GraphSLAM
ros2 launch tcc_slam_bringup navigation.launch.py \
  map:=2 slam_method:=graph

# Mundo 1 com MRPT RBPF-SLAM
ros2 launch tcc_slam_bringup navigation.launch.py \
  map:=1 slam_method:=rbpf

# Mundo 2 com MRPT EKF-SLAM
ros2 launch tcc_slam_bringup navigation.launch.py \
  map:=2 slam_method:=ekf
```

### Mundos disponíveis

| `map` | Arquivo | Nome interno Gazebo |
| --- | --- | --- |
| `1` | `tcc_world.sdf` | `tcc_world` |
| `2` | `tcc_world_2.sdf` | `tcc_world_2` |

Os arquivos ficam em `src/tcc_slam_description/worlds/`. Para incluir outro ambiente, adicione o SDF e registre seu identificador, nome de arquivo e nome interno em `MAPAS`, no `simulation.launch.py`. Recompile ao modificar arquivos instalados pelos pacotes.

## Executar um experimento autônomo

Com o sistema de navegação inicializado, abra outro terminal:

```bash
source /opt/ros/lyrical/setup.bash
source ~/tcc_slam_ws/install/setup.bash

python3 ~/tcc_slam_ws/executar_experimento.py 17 \
  --duracao 180 \
  --metodo graph
```

- `17`: identificador numérico do experimento (deve estar disponível);
- `--duracao 180`: duração em **segundos simulados**;
- `--metodo graph`: algoritmo que já deve estar em execução no launcher.

O executor aguarda o Nav2, aciona a exploração, grava os tópicos de interesse e solicita a análise ao término. Ele **não inicia nem seleciona** o método SLAM: `--metodo` identifica o algoritmo empregado.

> **Estado do código contido no snapshot do repositório analisado:** o executor grava em `bags/experimento_017/` e os resultados em `results/experimento_017/`. A atualização posterior de organização introduz pastas como `exp_17_mtd_graph_time_180_map_2` e a opção `--mapa`, mas esse formato só deve ser documentado como ativo **depois que a atualização for aplicada ao repositório**. Até lá, o mundo selecionado no launcher deve ser registrado manualmente no protocolo experimental.

## Analisar e comparar experimentos

Reanalisar uma execução já gravada:

```bash
~/tcc_slam_ws/analisar_experimento.sh 17
```

O script recupera o método e a duração do `manifest.json` correspondente. Os experimentos precisam conter os metadados e a rosbag necessários à análise.

Comparar pelo menos duas execuções:

```bash
python3 ~/tcc_slam_ws/comparar_experimento.py 17 18 19 20 \
  --saida comparacao_200s
```

Os resultados comparativos são exportados para `results/comparacoes/`. Abra o arquivo `relatorio_comparativo.html` no navegador para consultar tabelas, gráficos e mapas finais.

### Métricas produzidas

| Dimensão | Indicadores |
| --- | --- |
| Localização | ATE translacional e angular, RMSE, estatísticas descritivas |
| Consistência de movimento | RPE em janelas de 1, 5 e 10 segundos |
| Mapeamento | Área conhecida, área livre/ocupada, expansão temporal e taxa média |
| Navegação | Metas enviadas, alcançadas, falhas e sem resultado |
| Eficiência | Área conhecida acrescentada por tempo e por distância |
| Apresentação | Trajetórias, gráficos de erro, mapa final e relatórios HTML |

O mapeamento é acompanhado pela grade de ocupação publicada em `/map`, e as trajetórias estimadas são comparadas à referência `/ground_truth/odom` após a sincronização e o alinhamento definidos pelo analisador.

## Cuidados para uma comparação científica válida

1. **Manter condições controladas:** mesmo robô, sensores, ambiente, duração configurada, versão do Frontier Explorer e parâmetros do Nav2 ao comparar métodos.
2. **Distinguir as avaliações:** na exploração autônoma, cada SLAM pode levar o robô por uma trajetória diferente. Os resultados medem o sistema integrado, não apenas o algoritmo de localização isoladamente.
3. **Registrar configurações:** identificar método, mapa, parâmetros, versão do código e duração efetivamente analisada.
4. **Repetir as execuções:** uma execução por método permite comparação descritiva, mas não sustenta inferências estatísticas robustas.
5. **Não confundir cobertura e qualidade geométrica:** uma maior área conhecida não garante paredes mais precisas; a avaliação quantitativa da geometria do mapa requer referência e métrica próprias.
6. **Controlar o tempo de preparação:** o GraphSLAM utiliza um procedimento de *bootstrap* anterior ao início da navegação; o protocolo deve considerar esse efeito na interpretação da cobertura inicial.

## Próximas etapas planejadas

- **Trajetória fixa por mundo:** executar rotas equivalentes com os quatro métodos para reduzir a influência da estratégia de exploração na avaliação de SLAM.
- **Incerteza controlada dos sensores:** introduzir perturbações no LiDAR, IMU e odometria para estudar robustez, preservando a rastreabilidade dos dados e dos referenciais TF.

Essas funcionalidades **ainda não estão apresentadas como implementadas** neste README.

## Código de terceiros e licenças

Este repositório contém pacotes MRPT e adaptações de integração. Consulte `third_party/MRPT_ROS_LICENSE.txt` e os arquivos de licença dos pacotes de origem. As licenças e declarações em `package.xml` devem ser revisadas e uniformizadas antes de publicar uma licença geral para todo o código próprio; alguns pacotes locais ainda apresentam campos de licença provisórios.

## Autor

**Erick Cesar**  
Engenharia Mecânica — Instituto Federal da Paraíba (IFPB)

Projeto acadêmico para estudo e comparação de métodos de SLAM em robôs móveis autônomos.
