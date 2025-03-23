import numpy as np
import matplotlib.pyplot as plt
import heapq
import itertools
import random
import math

# ---------------- Função A* com movimentos diagonais (8-conectado) ----------------

def a_star(grid, start, goal):
    """
    A* em grid com 8 vizinhos. Movimentos cardeais têm custo 1 e movimentos diagonais custo sqrt(2).
    start e goal devem ser tuplas (x,y) com índices inteiros.
    """
    neighbors = [((0,1), 1), ((0,-1), 1), ((1,0), 1), ((-1,0), 1),
                 ((1,1), math.sqrt(2)), ((1,-1), math.sqrt(2)), ((-1,1), math.sqrt(2)), ((-1,-1), math.sqrt(2))]
    
    def heuristic(a, b):
        return math.hypot(a[0]-b[0], a[1]-b[1])
    
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
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
        for (dxdy, move_cost) in neighbors:
            neighbor = (current[0] + dxdy[0], current[1] + dxdy[1])
            if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]:
                if grid[neighbor] == 1:  # obstáculo
                    continue
                tentative_g = gscore[current] + move_cost
                if neighbor in close_set and tentative_g >= gscore.get(neighbor, float('inf')):
                    continue
                if tentative_g < gscore.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g
                    fscore[neighbor] = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_heap, (fscore[neighbor], neighbor))
    return None

def tsp_order(points, start):
    """
    Resolve o TSP por força bruta para poucas bases usando distância Euclidiana.
    A ordenação é calculada a partir do ponto 'start' (que agora é o detection_endpoint).
    """
    best_order = None
    best_distance = float('inf')
    def dist(a, b):
        return math.hypot(a[0]-b[0], a[1]-b[1])
    for perm in itertools.permutations(points):
        if not perm:
            continue
        d = dist(start, perm[0])
        for i in range(len(perm)-1):
            d += dist(perm[i], perm[i+1])
        if d < best_distance:
            best_distance = d
            best_order = perm
    if best_order is None:
        return []
    return list(best_order)

# ---------------- Geração Aleatória do Ambiente ----------------

area_size = 10       
grid_dim = 10        
num_posts = 10

# Define o ponto de início do drone
drone_start = (0, 0)

grid = np.zeros((grid_dim, grid_dim), dtype=int)

# Gera posições aleatórias para postes, garantindo que o poste não seja na posição de início.
posts = set()
while len(posts) < num_posts:
    pos = (random.randint(0, grid_dim-1), random.randint(0, grid_dim-1))
    if pos == drone_start:
        continue  
    posts.add(pos)
posts = list(posts)
for pos in posts:
    grid[pos] = 1

def gerar_bases(qtd, excluidos):
    """Gera 'qtd' bases evitando posições já ocupadas (por postes ou outras bases)."""
    bases = []
    while len(bases) < qtd:
        pos = (random.randint(0, grid_dim-1), random.randint(0, grid_dim-1))
        if pos in excluidos or pos in bases:
            continue
        bases.append(pos)
    return bases

true_pattern1 = gerar_bases(3, posts)
true_pattern2 = gerar_bases(3, posts + true_pattern1)

# ---------------- Parâmetros de Detecção ----------------

detection_goal = (grid_dim-1, grid_dim-1)   
visual_range = 6.0                        # Alcance visual da RunCam 5 (em metros)
detection_stop_fraction = 0.8             # Interrompe a detecção ao percorrer 80% da rota

# ---------------- Rota de Detecção "Diagonal" evitando postes ----------------

# Calcula a rota de detecção usando A* com movimentos diagonais
detection_route = a_star(grid, drone_start, detection_goal)
if detection_route is None:
    raise Exception("Não foi possível calcular a rota de detecção evitando os postes.")

# Durante o percurso, o drone "detecta" bases se elas estiverem dentro do visual_range.
detected_bases = {}  
for i in range(3):
    detected_bases[f'P1_{i}'] = None
for i in range(3):
    detected_bases[f'P2_{i}'] = None

detection_endpoint = None
route_length = len(detection_route)
for idx, point in enumerate(detection_route):
    current_pos = point
    for idx_b, base in enumerate(true_pattern1):
        key = f'P1_{idx_b}'
        if detected_bases[key] is None:
            if math.hypot(current_pos[0]-base[0], current_pos[1]-base[1]) <= visual_range:
                detected_bases[key] = base
    for idx_b, base in enumerate(true_pattern2):
        key = f'P2_{idx_b}'
        if detected_bases[key] is None:
            if math.hypot(current_pos[0]-base[0], current_pos[1]-base[1]) <= visual_range:
                detected_bases[key] = base
    if all(v is not None for v in detected_bases.values()) or (idx/route_length) >= detection_stop_fraction:
        detection_endpoint = current_pos
        break
if detection_endpoint is None:
    detection_endpoint = detection_route[-1]

print("Padrão 1 (verdadeiro):", true_pattern1)
print("Padrão 2 (verdadeiro):", true_pattern2)
print("Bases detectadas:", detected_bases)
print("Ponto final da detecção:", detection_endpoint)

# ---------------- Planejamento das Rotas de Aterrissagem ----------------

def average_distance(bases, point):
    return np.mean([math.hypot(point[0]-b[0], point[1]-b[1]) for b in bases])

# Define qual padrão tem a média de distância menor a partir do detection_endpoint.
if average_distance(true_pattern1, detection_endpoint) <= average_distance(true_pattern2, detection_endpoint):
    chosen_pattern = true_pattern1
    alt_pattern = true_pattern2
    chosen_label = "Padrão 1"
else:
    chosen_pattern = true_pattern2
    alt_pattern = true_pattern1
    chosen_label = "Padrão 2"

# O ponto de partida para o planejamento de aterrissagem é o detection_endpoint.
detection_point = detection_endpoint  
landing_route_chosen = [detection_point] + tsp_order(chosen_pattern, detection_point)
landing_route_alt = [detection_point] + tsp_order(alt_pattern, detection_point)

print("Rota de aterrissagem para", chosen_label, ":", landing_route_chosen)
print("Rota de aterrissagem para o padrão alternativo:", landing_route_alt)

def compute_full_path(route, grid):
    full_path = []
    for i in range(len(route)-1):
        segment = a_star(grid, route[i], route[i+1])
        if segment is None:
            print("Não foi possível encontrar caminho entre", route[i], "e", route[i+1])
            break
        if i != 0:
            segment = segment[1:]
        full_path += segment
    return full_path

landing_path_chosen = compute_full_path(landing_route_chosen, grid)
landing_path_alt = compute_full_path(landing_route_alt, grid)

# ---------------- Representação Gráfica ----------------

plt.figure(figsize=(8,8))
plt.xlim(0, grid_dim)
plt.ylim(0, grid_dim)
plt.gca().set_aspect('equal')

# Plota os postes (centralizando cada célula com +0.5)
for idx, post in enumerate(posts):
    plt.scatter(post[0]+0.5, post[1]+0.5, color="black", marker="s", s=100, 
                label="Poste" if idx == 0 else "")

# Plota as bases verdadeiras dos dois padrões
for idx, base in enumerate(true_pattern1):
    plt.scatter(base[0]+0.5, base[1]+0.5, color="magenta", marker="^", s=100,
                label="Base Padrão 1" if idx==0 else "")
for idx, base in enumerate(true_pattern2):
    plt.scatter(base[0]+0.5, base[1]+0.5, color="orange", marker="s", s=100,
                label="Base Padrão 2" if idx==0 else "")

# Plota a rota de detecção (calculada via A* com 8-direções) – em verde tracejado
det_x = [p[0]+0.5 for p in detection_route]
det_y = [p[1]+0.5 for p in detection_route]
plt.plot(det_x, det_y, linestyle="--", color="green", linewidth=2, label="Rota de Detecção")

# Marca o ponto inicial e o fim da detecção
plt.scatter(drone_start[0]+0.5, drone_start[1]+0.5, color="blue", marker="o", s=100, label="Início")
plt.scatter(detection_point[0]+0.5, detection_point[1]+0.5, color="purple", marker="D", s=100, label="Fim da Detecção")

# Plota as rotas de aterrissagem: vermelho para o padrão escolhido e azul para o alternativo.
if landing_path_chosen:
    lc_x = [p[0]+0.5 for p in landing_path_chosen]
    lc_y = [p[1]+0.5 for p in landing_path_chosen]
    plt.plot(lc_x, lc_y, color="red", linewidth=2, label="Aterrissagem ("+chosen_label+")")
if landing_path_alt:
    la_x = [p[0]+0.5 for p in landing_path_alt]
    la_y = [p[1]+0.5 for p in landing_path_alt]
    plt.plot(la_x, la_y, color="blue", linewidth=2, label="Aterrissagem (Padrão Alternativo)")

plt.title("Missão do Drone: Detecção (via diagonal evitando postes) e Aterrissagem")
plt.xlabel("Coordenada X (m)")
plt.ylabel("Coordenada Y (m)")
plt.legend()
plt.grid(True)
plt.show()
