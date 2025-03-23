import numpy as np
import matplotlib.pyplot as plt
import heapq
import itertools
import random
import math
import streamlit as st

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

# ---------------- Função para resolver TSP (Força Bruta) ----------------
def tsp_order(points, start):
    """
    Resolve o TSP por força bruta para poucas bases usando distância Euclidiana.
    A ordenação é calculada a partir do ponto 'start' (detection_endpoint).
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

# ---------------- Funções para gerar bases e calcular distância média ----------------
def gerar_bases(qtd, excluidos):
    """Gera 'qtd' bases evitando posições já ocupadas (por postes ou outras bases)."""
    bases = []
    while len(bases) < qtd:
        pos = (random.randint(0, grid_dim-1), random.randint(0, grid_dim-1))
        if pos in excluidos or pos in bases:
            continue
        bases.append(pos)
    return bases

def average_distance(bases, point):
    return np.mean([math.hypot(point[0]-b[0], point[1]-b[1]) for b in bases])

# ---------------- Parâmetros do Ambiente ----------------
area_size = 10       
grid_dim = 10        
num_posts = 10

# Define o ponto de início do drone e o objetivo da detecção
drone_start = (0, 0)
detection_goal = (grid_dim-1, grid_dim-1)

# ---------------- Função Principal ----------------
def main():
    max_attempts = 10
    attempt = 0
    detection_route = None
    
    # Tenta gerar um ambiente com uma rota viável até max_attempts vezes
    while attempt < max_attempts:
        # Cria um novo grid
        grid = np.zeros((grid_dim, grid_dim), dtype=int)
        
        # Gera posições aleatórias para postes (excluindo drone_start e detection_goal)
        posts = set()
        while len(posts) < num_posts:
            pos = (random.randint(0, grid_dim-1), random.randint(0, grid_dim-1))
            if pos == drone_start or pos == detection_goal:
                continue  
            posts.add(pos)
        posts = list(posts)
        for pos in posts:
            grid[pos] = 1

        # Gera os padrões verdadeiros das bases, evitando os postes.
        true_pattern1 = gerar_bases(3, posts)
        true_pattern2 = gerar_bases(3, posts + true_pattern1)

        # Parâmetros de detecção
        visual_range = 6.0                        
        detection_stop_fraction = 0.8             

        # Tenta calcular a rota de detecção usando A*
        detection_route = a_star(grid, drone_start, detection_goal)
        if detection_route is not None:
            break  # Se encontrou a rota, sai do loop
        attempt += 1
        st.write(f"Tentativa {attempt} sem sucesso. Gerando novo ambiente...")

    if detection_route is None:
        raise Exception("Não foi possível calcular a rota de detecção evitando os postes após várias tentativas.")

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

    # Seleciona o padrão com a menor média de distância ao detection_endpoint.
    if average_distance(true_pattern1, detection_endpoint) <= average_distance(true_pattern2, detection_endpoint):
        chosen_pattern = true_pattern1
        alt_pattern = true_pattern2
        chosen_label = "Padrão 1"
    else:
        chosen_pattern = true_pattern2
        alt_pattern = true_pattern1
        chosen_label = "Padrão 2"


    # Planejamento das Rotas de Aterrissagem
    detection_point = detection_endpoint  
    landing_route_chosen = [detection_point] + tsp_order(chosen_pattern, detection_point)
    landing_route_alt = [detection_point] + tsp_order(alt_pattern, detection_point)

    def compute_full_path(route, grid):
        full_path = []
        for i in range(len(route)-1):
            segment = a_star(grid, route[i], route[i+1])
            if segment is None:
                st.write("Não foi possível encontrar caminho entre", route[i], "e", route[i+1])
                break
            if i != 0:
                segment = segment[1:]
            full_path += segment
        return full_path

    landing_path_chosen = compute_full_path(landing_route_chosen, grid)
    landing_path_alt = compute_full_path(landing_route_alt, grid)

    # Representação Gráfica usando figura explícita
    fig, ax = plt.subplots(figsize=(8,8))
    ax.set_xlim(0, grid_dim)
    ax.set_ylim(0, grid_dim)
    ax.set_aspect('equal')

    # Plota os postes
    for idx, post in enumerate(posts):
        ax.scatter(post[0]+0.5, post[1]+0.5, color="black", marker="s", s=100, 
                   label="Poste" if idx == 0 else "")

    # Plota as bases dos padrões
    for idx, base in enumerate(true_pattern1):
        ax.scatter(base[0]+0.5, base[1]+0.5, color="magenta", marker="^", s=100,
                   label="Base Padrão 1" if idx==0 else "")
    for idx, base in enumerate(true_pattern2):
        ax.scatter(base[0]+0.5, base[1]+0.5, color="orange", marker="s", s=100,
                   label="Base Padrão 2" if idx==0 else "")

    # Plota a rota de detecção
    det_x = [p[0]+0.5 for p in detection_route]
    det_y = [p[1]+0.5 for p in detection_route]
    ax.plot(det_x, det_y, linestyle="--", color="green", linewidth=2, label="Rota de Detecção")

    # Marca o início e fim da detecção
    ax.scatter(drone_start[0]+0.5, drone_start[1]+0.5, color="blue", marker="o", s=100, label="Início")
    ax.scatter(detection_point[0]+0.5, detection_point[1]+0.5, color="purple", marker="D", s=100, label="Fim da Detecção")

    # Plota as rotas de aterrissagem
    if landing_path_chosen:
        lc_x = [p[0]+0.5 for p in landing_path_chosen]
        lc_y = [p[1]+0.5 for p in landing_path_chosen]
        ax.plot(lc_x, lc_y, color="red", linewidth=2, label="Aterrissagem ("+chosen_label+")")
    if landing_path_alt:
        la_x = [p[0]+0.5 for p in landing_path_alt]
        la_y = [p[1]+0.5 for p in landing_path_alt]
        ax.plot(la_x, la_y, color="blue", linewidth=2, label="Aterrissagem (Padrão Alternativo)")

    ax.set_title("Missão do Drone: Detecção (via diagonal evitando postes) e Aterrissagem")
    ax.set_xlabel("Coordenada X (m)")
    ax.set_ylabel("Coordenada Y (m)")
    ax.legend()
    ax.grid(True)
    
    st.pyplot(fig)
    plt.close(fig)

if __name__ == "__main__":
    main()
