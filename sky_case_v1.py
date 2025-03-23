import numpy as np
import matplotlib.pyplot as plt
import heapq
import itertools
from sklearn.cluster import DBSCAN

def manhattan_distance(a, b):
    """
    Calcula a distância Manhattan entre dois pontos.

    Parâmetros:
        a (tuple): Coordenadas do primeiro ponto (linha, coluna).
        b (tuple): Coordenadas do segundo ponto (linha, coluna).

    Retorna:
        int: Distância Manhattan entre os pontos a e b.
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(grid, start, goal):
    """
    Encontra o caminho mais curto em um grid utilizando o algoritmo A*.

    Parâmetros:
        grid (numpy.ndarray): Matriz representando o grid. Valor 1 indica obstáculo.
        start (tuple): Coordenadas do ponto de partida (linha, coluna).
        goal (tuple): Coordenadas do destino (linha, coluna).

    Retorna:
        list: Lista de coordenadas representando o caminho do início ao destino, ou None se não for possível encontrar o caminho.
    """
    neighbors = [(0,1), (0,-1), (1,0), (-1,0)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: manhattan_distance(start, goal)}
    open_heap = []
    heapq.heappush(open_heap, (fscore[start], start))
    
    while open_heap:
        current = heapq.heappop(open_heap)[1]
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
            
        close_set.add(current)
        for i, j in neighbors:
            neighbor = (current[0] + i, current[1] + j)
            if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]:
                if grid[neighbor] == 1:  # Abordagem de grafos que utilizei para representar um obstáculo
                    continue
                tentative_g_score = gscore[current] + 1
                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, float('inf')):
                    continue
                if tentative_g_score < gscore.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + manhattan_distance(neighbor, goal)
                    heapq.heappush(open_heap, (fscore[neighbor], neighbor))
    return None


def tsp_order(points, start):
    """
    Resolve o Problema do Caixeiro Viajante (TSP) do jeito mais pragmatico e simples que consegui.

    Parâmetros:
        points (list): Lista de pontos (coordenadas) a serem visitados.
        start (tuple): Coordenadas do ponto de partida.

    Retorna:
        list: Sequência de pontos que resulta na menor rota total considerando a distância Manhattan.
    """
    best_order = None
    best_distance = float('inf')
    for perm in itertools.permutations(points):
        if not perm:
            continue
        dist = manhattan_distance(start, perm[0])
        for i in range(len(perm) - 1):
            dist += manhattan_distance(perm[i], perm[i+1])
        if dist < best_distance:
            best_distance = dist
            best_order = perm
    if best_order is None:
        return []
    return list(best_order)

def cluster_bases(bases, eps=8, min_samples=1):
    """
    Agrupa bases (pontos) utilizando o algoritmo DBSCAN.

    Parâmetros:
        bases (list): Lista de coordenadas das bases.
        eps (float): Distância máxima para que dois pontos sejam considerados vizinhos.
        min_samples (int): Número mínimo de pontos para formar um cluster.

    Retorna:
        dict: Dicionário onde as chaves são os rótulos dos clusters e os valores são listas de bases pertencentes a cada cluster.
    """
    bases_array = np.array(bases)
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(bases_array)
    clusters = {}
    for label, base in zip(clustering.labels_, bases):
        clusters.setdefault(label, []).append(base)
    return clusters

def choose_cluster(clusters, start):
    """
    Escolhe o cluster cujas bases possuem a menor distância média (Manhattan) a partir do ponto de partida.

    Parâmetros:
        clusters (dict): Dicionário com rótulos de clusters e suas respectivas listas de pontos.
        start (tuple): Coordenadas do ponto de partida.

    Retorna:
        int ou None: Rótulo do cluster escolhido ou None se não houver clusters.
    """
    best_cluster = None
    best_score = float('inf')
    for label, points in clusters.items():
        score = np.mean([manhattan_distance(start, p) for p in points])
        if score < best_score:
            best_score = score
            best_cluster = label
    return best_cluster

grid_size = (20, 20)
grid = np.zeros(grid_size, dtype=int)
obstaculos = [(3,3), (3,4), (3,5), (10,2), (11,2), (12,2), (15,10), (16,10), (17,10)]
for obs in obstaculos:
    grid[obs] = 1

# Ponto de partida definido de forma fixa
start = (0, 0)

pattern1 = [(5, 5), (5, 15), (10, 10)]
pattern2 = [(15, 5), (15, 15), (10, 18)]
todas_bases = pattern1 + pattern2

clusters = cluster_bases(todas_bases, eps=8)
cluster_inicial = choose_cluster(clusters, start)
print("Clusters detectados:", clusters)
print("Cluster escolhido para iniciar:", cluster_inicial)

if cluster_inicial is not None:
    bases_primeiro_padrao = clusters[cluster_inicial]
    bases_segundo_padrao = []
    for label, pontos in clusters.items():
        if label != cluster_inicial:
            bases_segundo_padrao.extend(pontos)
else:
    bases_primeiro_padrao = pattern1
    bases_segundo_padrao = pattern2

ordem_primeiro = tsp_order(bases_primeiro_padrao, start)
if bases_segundo_padrao:
    ordem_segundo = tsp_order(bases_segundo_padrao, ordem_primeiro[-1])
else:
    ordem_segundo = []

rota_pontos = [start] + ordem_primeiro + ordem_segundo

caminho_completo = []
for i in range(len(rota_pontos) - 1):
    segmento = a_star(grid, rota_pontos[i], rota_pontos[i+1])
    if segmento is None:
        print("Não foi possível encontrar caminho entre", rota_pontos[i], "e", rota_pontos[i+1])
        break
    if i != 0:  # abordagem de grafos para nao revisitar ponto anterior e deixar marcado como lugar visitado
        segmento = segmento[1:]
    caminho_completo += segmento

plt.figure(figsize=(6, 6))
plt.imshow(grid, cmap="Greys", origin="lower")
caminho_x = [p[1] for p in caminho_completo]
caminho_y = [p[0] for p in caminho_completo]
plt.plot(caminho_x, caminho_y, color="red", label="Rota planejada")
plt.scatter(start[1], start[0], color="green", marker="o", s=100, label="Partida")

for base in ordem_primeiro:
    plt.scatter(base[1], base[0], color="blue", marker="^", s=100, label="Padrão Iniciado")
for base in ordem_segundo:
    plt.scatter(base[1], base[0], color="orange", marker="s", s=100, label="Outro Padrão")

# Evita legenda duplicada
handles, labels = plt.gca().get_legend_handles_labels()
by_label = dict(zip(labels, handles))
plt.legend(by_label.values(), by_label.keys())
plt.title("Roteirização Dinâmica com Clusterização das Bases")
plt.xlabel("Coordenada X")
plt.ylabel("Coordenada Y")
plt.grid(True)
plt.show()
