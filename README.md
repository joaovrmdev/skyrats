# skyrats

## [Primeira abordagem](https://github.com/joaovrmdev/skyrats/blob/main/sky_case_v1.py)
Nessa etapa, estava apenas me preocupando em como resolver a etapa de grafos, ou seja, nessa etapa, eu fui bem pouco disruptivo, apenas explorei o problema conceitualmente e tentei gerar representações visuais minimas.
Issues:
-> Não considerava posições aleatorias. 
-> Apenas um tipo de base.
-> Não foquei na delimitação da area de forma exata.

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


