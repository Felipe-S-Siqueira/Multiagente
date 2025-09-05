from dataclasses import dataclass, field
from typing import List, Tuple, Set, Deque, Optional
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from collections import deque
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

# ---------------- CONFIGURAÇÕES ----------------
GRID_SIZE = 10       # tamanho do grid (GRID_SIZE x GRID_SIZE)
NUM_OBSTACLES = 18   # quantidade de obstáculos
NUM_POINTS = 10      # quantidade de pontos de interesse
NUM_ROBOTS = 3       # quantidade de robôs
STEPS = 60           # quantidade de passos de simulação
RECENT_MEMORY = 10   # posições recentes para evitar voltar
RANDOM_SEED = None   # None = mapa aleatório, ou coloque int para reprodução
# ------------------------------------------------

if RANDOM_SEED is not None:
    random.seed(RANDOM_SEED)
    np.random.seed(RANDOM_SEED)

global_visited_points: Set[Tuple[int, int]] = set()

# ---------------- CLASSES ----------------
class Environment:
    def __init__(self, size, num_obstacles, num_points):
        self.size = size
        self.grid = np.zeros((size, size), dtype=int)  # -1 obst, 0 livre, 1 ponto
        self._place_random(num_obstacles, -1)
        self._place_random(num_points, 1, forbidden={-1})

    def _place_random(self, n, val, forbidden: Set[int]=None):
        forbidden = forbidden or set()
        placed = 0
        while placed < n:
            i, j = random.randrange(self.size), random.randrange(self.size)
            if self.grid[i, j] == 0 and self.grid[i, j] not in forbidden:
                self.grid[i, j] = val
                placed += 1

    def is_free(self, pos: Tuple[int,int]) -> bool:
        i,j = pos
        return 0 <= i < self.size and 0 <= j < self.size and self.grid[i,j] != -1

@dataclass
class RoboAgente:
    id: int
    pos: Tuple[int,int]
    env: Environment
    visited: Set[Tuple[int,int]] = field(default_factory=set)
    recent: Deque[Tuple[int,int]] = field(default_factory=lambda: deque(maxlen=RECENT_MEMORY))
    local_map: np.ndarray = field(default_factory=lambda: np.full((GRID_SIZE, GRID_SIZE), 9, dtype=int))
    visited_points: Set[Tuple[int,int]] = field(default_factory=set)
    plan: Deque[Tuple[int,int]] = field(default_factory=deque)

    # vizinhos diretos (cima, baixo, esquerda, direita)
    def perceive_neighbors(self) -> List[Tuple[int,int]]:
        i,j = self.pos
        neighbors = [(i-1,j),(i+1,j),(i,j-1),(i,j+1)]
        return [(x,y) for x,y in neighbors if 0<=x<self.env.size and 0<=y<self.env.size]

    # observa entorno e compartilha mapa global
    def observe_and_share(self, global_map: np.ndarray):
        for x,y in ([self.pos] + self.perceive_neighbors()):
            v = self.env.grid[x,y]
            self.local_map[x,y] = v
            global_map[x,y] = v

    # encontra caminho até próximo alvo desconhecido ou ponto não visitado
    def find_nearest_target_path(self, global_map: np.ndarray) -> Optional[List[Tuple[int,int]]]:
        start = self.pos
        q = deque([[start]])
        seen = {start}
        while q:
            path = q.popleft()
            cur = path[-1]
            if cur != start and (global_map[cur] == 9 or (global_map[cur] == 1 and cur not in global_visited_points)):
                return path
            for nb in [(cur[0]-1,cur[1]),(cur[0]+1,cur[1]),(cur[0],cur[1]-1),(cur[0],cur[1]+1)]:
                if 0<=nb[0]<self.env.size and 0<=nb[1]<self.env.size and nb not in seen and self.env.is_free(nb):
                    seen.add(nb)
                    q.append(path + [nb])
        return None

    # gera lista de candidatos ordenada por prioridade
    def ranked_candidates(self, global_map: np.ndarray) -> List[Tuple[int,int]]:
        base = [self.plan[0]] + [n for n in self.perceive_neighbors() if n != self.plan[0]] if self.plan else self.perceive_neighbors()
        if self.pos not in base:
            base.append(self.pos)
        scored = []
        for n in base:
            if not self.env.is_free(n):
                continue
            val = global_map[n]
            s = 0
            if val == 9: s += 120
            elif val == 1: s += 70 if n not in global_visited_points else 10
            elif val == 0: s += 40 if n not in self.visited else 5
            if n == self.pos: s -= 40
            if n in self.recent: s -= 30
            s += random.random()
            scored.append((s, n))
        if not scored:
            return [self.pos]
        scored.sort(key=lambda x: x[0], reverse=True)
        return [p for _, p in scored]

    # executa movimento e atualiza mapa local/global
    def step_commit(self, new_pos: Tuple[int,int], global_map: np.ndarray):
        global global_visited_points
        if self.env.is_free(new_pos):
            if self.plan and self.plan[0] == new_pos:
                self.plan.popleft()
            self.pos = new_pos
        self.visited.add(self.pos)
        self.recent.append(self.pos)
        self.observe_and_share(global_map)
        if self.env.grid[self.pos] == 1:
            self.visited_points.add(self.pos)
            global_visited_points.add(self.pos)
            self.local_map[self.pos] = 1
            global_map[self.pos] = 1

# ---------------- INICIALIZAÇÃO ----------------
env = Environment(GRID_SIZE, NUM_OBSTACLES, NUM_POINTS)
global_map = np.full((GRID_SIZE, GRID_SIZE), 9, dtype=int)

def random_free_cell(taken:Set[Tuple[int,int]]):
    while True:
        i,j = random.randrange(GRID_SIZE), random.randrange(GRID_SIZE)
        if env.grid[i,j] != -1 and (i,j) not in taken:
            return (i,j)

robots: List[RoboAgente] = []
taken = set()
for rid in range(NUM_ROBOTS):
    s = random_free_cell(taken)
    taken.add(s)
    r = RoboAgente(id=rid, pos=s, env=env)
    r.visited.add(s)
    r.recent.append(s)
    r.observe_and_share(global_map)
    robots.append(r)

# atualiza mapa global com informações de todos os robôs
def broadcast_merge(robots: List[RoboAgente], global_map: np.ndarray):
    for r in robots:
        mask = (r.local_map != 9)
        global_map[mask] = r.local_map[mask]
        global_visited_points.update(r.visited_points)

# ---------------- VISUALIZAÇÃO ----------------
value_to_index = {-1:0,0:1,1:2,9:3}
cmap = mcolors.ListedColormap(['red','white','green','lightgray'])
norm = mcolors.BoundaryNorm([0,1,2,3,4], cmap.N)
def map_for_display(g): return np.vectorize(lambda v: value_to_index.get(int(v),3))(g)

fig, ax = plt.subplots(figsize=(6,6))
im = ax.imshow(map_for_display(global_map), cmap=cmap, norm=norm)
colors = ["tab:blue","tab:orange","tab:purple"]
scatters = [ax.scatter(r.pos[1], r.pos[0], c=[colors[i%len(colors)]], s=140, edgecolors="black", zorder=5)
            for i,r in enumerate(robots)]

# legenda
legend = [
    Patch(facecolor='red', edgecolor='k', label='Obstáculos'),
    Patch(facecolor='white', edgecolor='k', label='Livres'),
    Patch(facecolor='green', edgecolor='k', label='Pontos'),
    Patch(facecolor='lightgray', edgecolor='k', label='Desconhecido')
] + [Line2D([0],[0], marker='o', color='w', label=f'Robo {i+1}',
            markerfacecolor=colors[i%len(colors)], markersize=8, markeredgecolor='k')
     for i in range(NUM_ROBOTS)]
ax.legend(handles=legend, bbox_to_anchor=(1.02,1.0), loc='upper left')
ax.set_xticks([]); ax.set_yticks([])
ax.set_title("Exploração Multiagente — Mapa Global (descoberto)")
plt.tight_layout()

# ---------------- CONTROLE E ANIMAÇÃO ----------------
def resolve_conflicts_and_move(robots, global_map):
    for r in robots: r.observe_and_share(global_map)
    for r in robots:
        if not r.plan:
            path = r.find_nearest_target_path(global_map)
            if path and len(path) >= 2:
                r.plan = deque(path[1:])
    # seleção de movimento para cada robô
    ranked_lists, candidates = {}, {}
    for r in robots:
        ranked_lists[r.id] = r.ranked_candidates(global_map)
        candidates[r.id] = ranked_lists[r.id][0] if ranked_lists[r.id] else r.pos
    # resolução de conflitos
    target_to_ids, assigned, occupied = {}, {}, set()
    for rid, target in candidates.items(): target_to_ids.setdefault(target, []).append(rid)
    for target, ids in target_to_ids.items():
        if len(ids) == 1:
            assigned[ids[0]] = target
            occupied.add(target)
    for target, ids in target_to_ids.items():
        if len(ids) > 1:
            for i, rid in enumerate(sorted(ids)):
                if rid in assigned: continue
                chosen = None
                if i == 0 and target not in occupied: chosen = target
                else:
                    for cand in ranked_lists[rid]:
                        if cand not in occupied: chosen = cand; break
                    if chosen is None: chosen = [r for r in robots if r.id==rid][0].pos
                assigned[rid] = chosen
                occupied.add(chosen)
    for r in robots: r.step_commit(assigned.get(r.id, r.pos), global_map)
    broadcast_merge(robots, global_map)

def update(frame):
    resolve_conflicts_and_move(robots, global_map)
    im.set_data(map_for_display(global_map))
    for i,r in enumerate(robots):
        scatters[i].set_offsets([[r.pos[1], r.pos[0]]])
    num_found = len(global_visited_points)
    ax.set_title(
        f"Passos {frame+1}/{STEPS} — "
        f"Área mapeada: {np.sum(global_map != 9)}/{GRID_SIZE*GRID_SIZE} — "
        f"Pontos encontrados: {num_found}/{NUM_POINTS}"
    )
    return [im] + scatters

ani = FuncAnimation(fig, update, frames=STEPS, interval=250, blit=False, repeat=False)
plt.show()

# ---------------- RESULTADOS ----------------
print("\nAmbiente (ground truth):\n", env.grid)
print("\nMapa global construído:\n", global_map)
print("\nPontos encontrados:", sorted(global_visited_points))
