# Comparação de experimentos do TCC-SLAM

## Instalação

Copie `comparar_experimento.py` para `/home/erick/tcc_slam_ws/comparar_experimento.py`.

```bash
cp ~/Downloads/comparar_experimento.py ~/tcc_slam_ws/comparar_experimento.py
chmod +x ~/tcc_slam_ws/comparar_experimento.py
```

Requer Python 3, pandas, numpy e matplotlib, normalmente disponíveis na instalação usada pelo analisador. Caso necessário:

```bash
sudo apt install python3-pandas python3-numpy python3-matplotlib
```

Não precisa executar `colcon build`, pois o comparador é um script Python avulso, fora dos pacotes ROS.

## Executar

Para quatro experimentos já analisados:

```bash
python3 ~/tcc_slam_ws/comparar_experimento.py 21 22 23 24
```

Também é possível fornecer um nome de pasta:

```bash
python3 ~/tcc_slam_ws/comparar_experimento.py 21 22 23 24 --saida comparacao_200s
```

O relatório será criado em:

`~/tcc_slam_ws/results/comparacoes/comparacao_.../relatorio_comparativo.html`

Abrir no Firefox:

```bash
firefox ~/tcc_slam_ws/results/comparacoes/comparacao_200s/relatorio_comparativo.html
```

O script usa os arquivos existentes em `results/experimento_XXX/`, sem precisar executar ROS 2, Gazebo, RViz2 nem ler rosbag. Para cada experimento é obrigatório que exista `metricas.csv` (gerado por `analisar_experimento.sh X`). Os demais CSVs e o mapa final são opcionais: gráficos correspondentes serão omitidos quando dados não estiverem disponíveis.

## Saídas

- `relatorio_comparativo.html`: relatório visual comparativo.
- `metricas_comparadas.csv`: linha por experimento, preservando todos os indicadores originais.
- `metricas_por_indicador.csv`: formato longo, uma linha por indicador e experimento.
- `configuracao.json`: identificadores, janela comum e avisos metodológicos.
- `graficos/*.png`: gráficos de ATE, RPE, cobertura, exploração e mapas finais individuais, quando disponíveis.

## Comparabilidade

- O script **não altera** as métricas individuais já calculadas. As barras mostram os resultados finais de cada execução.
- Para gráficos temporais, limita visualização ao menor período observado dentre os experimentos; **não recalcula** métricas finais nesse período comum.
- Quando durações forem diferentes, apresenta aviso; indicadores finais de cobertura e exploração não recebem destaque de melhor valor.
- As imagens de mapa são exibidas lado a lado, sem serem sobrepostas ou comparadas célula a célula, pois os referenciais podem diferir.
- Uma execução por método permite comparação **descritiva**, não inferência estatística. Para conclusões acadêmicas robustas, mantenha ambiente, ruído, duração e condições iniciais comparáveis e use repetições.

## Testes

O código foi validado por compilação de sintaxe e execução sobre CSVs sintéticos; o funcionamento com as gravações reais deverá ser confirmado no Ubuntu do projeto.
