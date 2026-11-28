# TCC — Comparação de SLAM com ROS 2 Lyrical, MRPT e Nav2

Workspace de pesquisa: robô diferencial simulado no Gazebo, visualização RViz2, três implementações MRPT (GraphSLAM, RBPF-SLAM e EKF-SLAM), exploração por fronteiras e rotas fixas. Aquisição em rosbag2 e análise de métricas e mapas.

## Organização

- `src/tcc_slam_description/`: robô Xacro/URDF e mundos SDF (mapas 1–4).
- `src/tcc_slam_bringup/`: lançamento, bridges, Nav2, arquivos MRPT, prontidão e bootstrap.
- `src/tcc_slam_exploration/`: explorador de fronteiras e rota fixa; `routes/mapa_1.json` até `mapa_4.json`.
- `src/tcc_slam_ground_truth/`: posição de referência no simulador.
- `src/tcc_slam_analysis/`: processamento da rosbag e gráficos/relatórios.
- `src/mrpt_{graphslam_2d,rbpf_slam,ekf_slam_2d}/`: adaptações dos pacotes de terceiros.
- `executar_experimento.py`, `executar_bateria_tcc.py`, `worker_experimento_terminal.py`: experimento individual e bateria (uma janela de terminal por teste).
- `analisar_experimento.sh`, `comparar_experimento.py`, `consistencia_repeticoes.py`, `exportar_pdf.py`, `experimento_paths.py`: análise, comparações e relatórios.

**Preserve `bags/` e `results/`: são dados científicos, não arquivos descartáveis.** `logs/`, `backups/`, `build/`, `install/`, `log/` e `__pycache__/` são artefatos operacionais. Não remova `.git/`: contém histórico e alterações versionadas.

## Instalação e compilação

Requer Ubuntu 26.04, ROS 2 Lyrical, Gazebo Sim, ROS-Gazebo Bridge, Nav2, MRPT, Python com Pandas/NumPy/Matplotlib e dependências declaradas nos `package.xml`. As adaptações C++ requerem recompilação; **não basta copiar o código atualizado para `src/`**.

```bash
source /opt/ros/lyrical/setup.bash
cd ~/tcc_slam_ws
colcon build --symlink-install --parallel-workers 1 --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

Para verificar arquivos Python após atualizações:

```bash
python3 -m py_compile executar_bateria_tcc.py worker_experimento_terminal.py executar_experimento.py
```

## Experimento individual

Terminal 1:

```bash
source /opt/ros/lyrical/setup.bash
source ~/tcc_slam_ws/install/setup.bash
ros2 launch tcc_slam_bringup navigation.launch.py map:=2 slam_method:=graph
```

Terminal 2 (depois de Nav2 ficar ativo):

```bash
source /opt/ros/lyrical/setup.bash
source ~/tcc_slam_ws/install/setup.bash
python3 ~/tcc_slam_ws/executar_experimento.py 300 --metodo graph --mapa 2 --duracao 180 --movimentacao fronteiras
```

Para rota fixa, utilize `--movimentacao rota_fixa`. Nesse modo, `--duracao` não determina o encerramento: a execução termina quando a rota é concluída ou quando há falha.

## Bateria em terminais separados

Terminal principal:

```bash
python3 ~/tcc_slam_ws/executar_bateria_tcc.py \
  --inicio 400 --metodos graph rbpf ekf --mapas 1 2 3 4 \
  --movimentacao ambas --repeticoes 3 --duracao 180 --simular
```

Remova `--simular` para executar de verdade. O terminal principal abre um terminal gráfico por experimento, inicializa `navigation.launch.py`, espera Nav2 ativo, executa `executar_experimento.py`, encerra os subprocessos e abre o próximo terminal.

- `--retomar`: ignora **somente** IDs com bag, manifest e resultados válidos. Resultados incompletos não são sobrescritos.
- `--tentativas-inicializacao 2`: reinicia um teste em outro terminal **somente** quando falhou antes da primeira meta; pastas parciais de tentativas são preservadas sob logs/dados_parciais/; uma falha durante a navegação não é repetida automaticamente.
- `--pausa 12`: segundos reais de intervalo entre terminais.
- `--timeout-prontidao 240`, `--timeout-fronteiras 3600`, `--timeout-rota 7200`: limites em segundos reais para detectar travamentos.
- `--sem-comparacoes`, `--comparar-metodos`: opções de avaliação; `--somente-comparar` não executa simulações.

A bateria para na primeira falha real de navegação para **não adulterar a amostra**. Logs de cada tentativa ficam em `logs/baterias_terminais/`.

## Análise e integridade

```bash
~/tcc_slam_ws/analisar_experimento.sh 300
python3 ~/tcc_slam_ws/comparar_experimento.py 300 301 302
```

Para cada combinação, compare apenas resultados com mesmo mapa, modo de movimentação e (para fronteiras) tempo simulado configurado; para rotas fixas, preserve o hash SHA-256 do JSON. A bateria não altera pontos de rota. A configuração do Nav2 é compartilhada pelos três métodos para garantir condições equivalentes.

**Mudanças desta revisão:**
- `controller_frequency`: 20 → 10 Hz, mantendo `FollowPath.model_dt=0.05` s (dentro da regra do MPPI de não usar `model_dt` maior que o período de controle).
- `transform_tolerance`: 0,3 s nos costmaps local e global; a tolerância do `controller_server` é herdada do costmap local, não um parâmetro próprio declarado no nó.
- O GraphSLAM data a transformação `map → odom` 100 ms à frente do relógio simulado para evitar pequenas extrapolações futuras; valores de pose e timestamps das observações não mudam.
- Removida a serialização repetida do histórico inteiro de odometria 20 vezes/s; publicações de diagnóstico do GraphSLAM reduzidas, sem modificar o grafo nem as métricas do rosbag.
- Removido o caminho ativo obsoleto do SLAM Toolbox. O código de análise ainda reconhece o identificador `toolbox` em resultados legados.

Esses ajustes podem alterar resultados; portanto, **não misture repetições feitas antes e depois da revisão sem identificar a versão da configuração**.

## Limpeza segura

A ferramenta `limpar_workspace.sh` (fornecida no pacote de patch) mostra o que seria removido; só apaga arquivos com `--aplicar`. Preserva sempre `.git/`, `bags/` e `results/`. `build/` e `install/` só são removidos caso você peça `--limpar-build` e será necessário recompilar.

## Limites de validação

Este pacote passou por validação sintática e testes automatizados offline. **Ele não foi executado no Ubuntu do usuário com ROS 2/Gazebo/Nav2 reais**, portanto a estabilidade de dezenas de simulações consecutivas exige um teste local. Não é possível garantir ausência de falhas de ambiente ou de planejamento.

## Licenças

Os pacotes MRPT derivam de terceiros. Consulte `third_party/` e as licenças específicas nos pacotes antes de republicar o código.
