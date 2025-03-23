# skyrats

A abordagem utilizada nessa resolução preve o uso das duas cameras RunCam 5 e os sensores de detecção para detecção dos postes.

Uma representação das versões está disponivel via streamlit, clique aqui para ver a estrategia em ação.

## [Primeira abordagem](https://github.com/joaovrmdev/skyrats/blob/main/sky_case_v1.py)
Nessa etapa, estava apenas me preocupando em como resolver a etapa de grafos, ou seja, nessa etapa, eu fui bem pouco disruptivo, apenas explorei o problema conceitualmente e tentei gerar representações visuais minimas.
Issues:
+ Não considerava posições aleatorias. 
+ Apenas um tipo de base.
+ Não foquei na delimitação da area de forma exata.

Resumidamente, um grafo bem grosseiro haha.

### Clusterização das Bases
Este projeto implementa uma abordagem para planejamento de rotas em um grid, utilizando algoritmos clássicos como o A* para busca de caminho, o problema do caixeiro viajante (TSP) resolvido por força bruta para definir a ordem de visita, e a clusterização das bases utilizando o algoritmo DBSCAN.

### Funcionalidades

- **Busca de Caminho (A\*)**  
  Utiliza o algoritmo A* para encontrar o caminho mais curto entre dois pontos em um grid. Obstáculos são representados por células com valor 1.

- **Cálculo de Distância**  
  São implementadas funções para cálculo da distância Manhattan, tanto para o algoritmo A* quanto para a avaliação das rotas no TSP.

- **Planejamento de Rota (TSP)**  
  Determina a ordem de visita das bases (pontos) utilizando uma abordagem de força bruta, adequada para conjuntos pequenos de pontos.

- **Clusterização de Bases**  
  Agrupa as bases em clusters utilizando o algoritmo DBSCAN e, a partir disso, escolhe o cluster com menor distância média em relação ao ponto de partida para iniciar a rota.

### Descrição das Funções

- `heuristic(a, b)`  
  Calcula a distância Manhattan entre dois pontos, servindo como heurística para o algoritmo A*.

- `a_star(grid, start, goal)`  
  Implementa o algoritmo A* para busca de caminho em um grid, considerando os obstáculos definidos.

- `manhattan_distance(a, b)`  
  Retorna a distância Manhattan entre dois pontos, utilizada em outras funções para cálculos de distância.

- `tsp_order(points, start)`  
  Resolve o problema do caixeiro viajante para determinar a sequência ótima de visita dos pontos, minimizando a distância total (utiliza força bruta).

- `cluster_bases(bases, eps, min_samples)`  
  Agrupa as bases em clusters usando o algoritmo DBSCAN, permitindo separar diferentes conjuntos de pontos.

- `choose_cluster(clusters, start)`  
  Seleciona o cluster cujas bases possuem a menor distância média em relação ao ponto de partida, auxiliando na escolha do caminho inicial.

### Fluxo de Execução

1. **Configuração do Grid e Obstáculos:**  
   Define um grid 20x20 e marca células que representam obstáculos.

2. **Definição de Padrões de Bases e Ponto de Partida:**  
   São definidos dois conjuntos (padrões) de bases e um ponto inicial.

3. **Clusterização e Seleção do Cluster Inicial:**  
   As bases são agrupadas e o cluster mais próximo do ponto de partida é escolhido.

4. **Determinação da Ordem de Visita (TSP):**  
   Para o cluster selecionado e o restante das bases, calcula-se a ordem de visita que minimiza a distância total.

5. **Geração da Rota Completa com A\*:**  
   Utilizando o algoritmo A*, é calculado o caminho entre cada par consecutivo de pontos na rota final.

6. **Visualização:**  
   O grid, obstáculos, bases e a rota são plotados utilizando a biblioteca Matplotlib.

Grafico gerado (fixo):

![sky case v1](https://github.com/joaovrmdev/skyrats/blob/main/img/sky_case_v1.png)

---


## [Segunda abordagem](https://github.com/joaovrmdev/skyrats/blob/main/sky_case_v2.py)
Esta versão do projeto simula a missão de um drone em um ambiente gerado aleatoriamente, onde o drone realiza:

- **Rota de Detecção:** Utilizando o algoritmo A* (com movimentos diagonais) para encontrar o caminho entre dois pontos (de início até o canto oposto do grid), enquanto "detecta" bases próximas durante o percurso.
- **Planejamento de Aterrissagem:** Após a detecção, o drone planeja as rotas de aterrissagem para dois conjuntos de bases (padrões) utilizando uma abordagem ingênua para o Problema do Caixeiro Viajante (TSP) baseada em força bruta e distância Euclidiana.
- **Visualização:** Exibição do grid, postes (obstáculos), bases, rota de detecção e os caminhos de aterrissagem utilizando Matplotlib.

Nessa abordagem, o drone percorre a diagonal e so depois de percorrer ela, ele volta e começa o cenario de aterrisagem.
Features:
+ Um pouco mais disruptivo;
+ Abordgaem com duas bases e dinamico;
+ Segmentação de trechos

Issues:
- Essa abordagem faria com que ao terminar a aterrisagem da primeira base, o drone retorne ao inicio para seguir para a proxima;
- Pouco compreensivo quanto a qual das bases foi a primeira a ser seguida.
  

## Funcionalidades

1. **Algoritmo A\* com Movimentos Diagonais:**
   - Permite trajetórias mais realistas, considerando movimentos em 8 direções (cima, baixo, esquerda, direita e diagonais) com custos diferenciados (1 para movimentos cardeais e √2 para diagonais).

2. **Resolução do TSP (Força Bruta):**
   - Define a ordem ótima de visita para as bases utilizando a distância Euclidiana.
   - O cálculo da rota parte do ponto onde a detecção foi concluída (detection_endpoint).

3. **Geração Aleatória do Ambiente:**
   - O grid possui dimensões de 10x10 (cada célula representando 1 metro).
   - São gerados 10 postes (obstáculos) de forma aleatória, garantindo que a posição de início (fixa nesta versão) esteja livre.
   - Bases para dois padrões (true_pattern1 e true_pattern2) são geradas aleatoriamente, evitando colisões com os postes.

4. **Detecção Durante o Percurso:**
   - Enquanto o drone percorre a rota de detecção (calculada pelo A*), ele “detecta” bases se estas estiverem a uma distância menor ou igual a 4 metros (visual_range).
   - A detecção é interrompida quando todas as 6 bases forem detectadas ou quando 80% da rota for percorrida.

5. **Planejamento e Execução das Rotas de Aterrissagem:**
   - Compara-se a média das distâncias (Euclidianas) dos dois padrões a partir do ponto de término da detecção para escolher o padrão mais próximo.
   - São calculadas as rotas para o padrão escolhido e para o padrão alternativo, a partir do mesmo ponto de partida (detection_endpoint), e os caminhos completos são concatenados utilizando o A*.

6. **Visualização:**
   - Exibição gráfica do ambiente com os postes, bases, rota de detecção (linha verde tracejada) e as rotas de aterrissagem (em vermelho para o padrão escolhido e azul para o alternativo).

Exemplo do gráfico gerado (Dinamico):
![sky case v1](https://github.com/joaovrmdev/skyrats/blob/main/img/sky_case_v2.png)

---

## [Terceira abordagem](https://github.com/joaovrmdev/skyrats/blob/main/sky_case_v3.py)
### Missão do Drone: Detecção e Aterrissagem com Ambiente Aleatório

Aqui já estamos trabalhando com o cenario real do case. Temos start nos 4 vertices possiveis, e a minha última abordagem disruptiva para melhor performance.
Features:
+ Além da abordagem com a diagonal, agora usamos o metodo **diagonal ou truco**, se o drone registra as 6 bases durante o percurso diagonal, ele truca e inicia dessa posição mesmo, garantindo um pouco de ganho de tempo.
+ Adendo final:
Estou assumindo uma implementação das cameras (RunCam 5) com acuidade visual de 170º para cada uma; estas estariam dispostas na frente e atrás do drone. Assumi ainda, conforme veremos que o drone consegue identificar a base estando a 6 metros de distancia.

### Funcionalidades e Melhorias

### 1. Algoritmo A* com Movimentos Diagonais
- **Melhoria:** Suporte a movimentos em 8 direções com custos diferenciados:
  - Movimentos cardeais (cima, baixo, esquerda, direita) com custo 1.
  - Movimentos diagonais com custo `sqrt(2)`.
- **Objetivo:** Permitir trajetórias mais realistas no grid.

### 2. Resolução do TSP via Força Bruta
- **Melhoria:** Utilização da distância Euclidiana (via `math.hypot`) para calcular as rotas mais curtas entre pontos.
- **Objetivo:** Planejar a sequência de visita para as bases de forma ótima para pequenos conjuntos.

### 3. Geração Aleatória do Ambiente
- **Características:**
  - Grid 10x10 representando uma área de 10m x 10m.
  - Postes (obstáculos) gerados aleatoriamente, garantindo que a posição de início do drone esteja livre.
  - Geração aleatória de dois padrões de bases, evitando colisão com os postes.

### 4. Mecanismo de Detecção
- **Detalhes:**
  - Rota de detecção calculada pelo A* que vai do ponto inicial até o canto oposto do grid.
  - Durante o percurso, o drone detecta bases que estejam a uma distância menor ou igual ao `visual_range` (6 metros).
  - A detecção é interrompida quando 6 bases são detectadas ou após 80% da rota.

### 5. Planejamento das Rotas de Aterrissagem
- **Fluxo:**
  - Seleção do padrão de bases com menor distância média em relação ao ponto onde a detecção foi finalizada.
  - Utilização do TSP para definir a ordem de visita para cada grupo.
  - Cálculo do caminho completo entre os pontos utilizando o A*.
  
### 6. Visualização do Ambiente
- **Recursos:**
  - Exibição do grid com os postes, bases e as rotas traçadas.
  - Diferenciação das rotas: rota de detecção (verde tracejada) e trechos de aterrissagem (em vermelho e azul).

Exemplo do gráfico gerado (Dinamico):
![sky case v3](https://github.com/joaovrmdev/skyrats/blob/main/img/sky_case_v3.png)

---
---
