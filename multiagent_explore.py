from dataclasses import dataclass, field
from typing import List, Tuple, Set, Deque, Optional
import random, numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from collections import deque
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

# ---------------- configs (ajuste aqui) ----------------
GRID_SIZE = 10 # tamanho do grid (GRID_SIZE x GRID_SIZE)
NUM_OBSTACLES = 18 # quantos obstáculos
NUM_POIS = 10 # pontos de interesse
NUM_ROBOTS = 3 # quantos robôs
STEPS = 60 # quantos passos de simulação
RECENT_MEMORY = 10  # quantas posições recentes lembrar para evitar voltar
RANDOM_SEED = None   # None = mapa aleatório a cada execução; ou coloque int para reprodução
# -------------------------------------------------------

if RANDOM_SEED is not None:
    random.seed(RANDOM_SEED); np.random.seed(RANDOM_SEED)

global_visited_pois: Set[Tuple[int, int]] = set()

class Environment:
    def __init__(self, size, num_obstacles, num_pois):
        self.size = size
        self.grid = np.zeros((size, size), dtype=int)  # -1 obst, 0 livre, 1 POI
        self._place_random(num_obstacles, -1)
        self._place_random(num_pois, 1, forbidden={-1})

    def _place_random(self, n, val, forbidden: Set[int]=None):
        placed = 0; forbidden = forbidden or set()
        while placed < n:
            i, j = random.randrange(self.size), random.randrange(self.size)
            if self.grid[i, j] in forbidden: continue
            if self.grid[i, j] == 0:
                self.grid[i, j] = val; placed += 1

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
    visited_pois: Set[Tuple[int,int]] = field(default_factory=set)
    plan: Deque[Tuple[int,int]] = field(default_factory=deque)

    def perceive_neighbors(self) -> List[Tuple[int,int]]:
        i,j = self.pos
        cand = [(i-1,j),(i+1,j),(i,j-1),(i,j+1)]
        return [(x,y) for x,y in cand if 0<=x<self.env.size and 0<=y<self.env.size]

    def observe_and_share(self, global_map: np.ndarray):
        for x,y in ([self.pos] + self.perceive_neighbors()):
            v = self.env.grid[x,y]; self.local_map[x,y] = v; global_map[x,y] = v

    def find_nearest_target_path(self, global_map: np.ndarray) -> Optional[List[Tuple[int,int]]]:
        start = self.pos; q = deque([[start]]); seen = {start}
        while q:
            path = q.popleft(); cur = path[-1]
            if cur != start and global_map[cur] == 9: return path
            if cur != start and global_map[cur] == 1 and cur not in global_visited_pois: return path
            for nb in [(cur[0]-1,cur[1]),(cur[0]+1,cur[1]),(cur[0],cur[1]-1),(cur[0],cur[1]+1)]:
                if 0<=nb[0]<self.env.size and 0<=nb[1]<self.env.size and nb not in seen and self.env.is_free(nb):
                    seen.add(nb); q.append(path+[nb])
        return None

    def ranked_candidates(self, global_map: np.ndarray) -> List[Tuple[int,int]]:
        if self.plan:
            plan_next = self.plan[0]; neighbors = self.perceive_neighbors(); base = [plan_next] + [n for n in neighbors if n!=plan_next]
        else:
            base = self.perceive_neighbors()
        if self.pos not in base: base.append(self.pos)
        scored=[]
        for n in base:
            if not self.env.is_free(n): continue
            val = global_map[n]; s=0
            if val==9: s+=120
            elif val==1: s+=70 if n not in global_visited_pois else 10
            elif val==0: s+=40 if n not in self.visited else 5
            if n==self.pos: s-=40
            if n in self.recent: s-=30
            s += random.random()*1.0
            scored.append((s,n))
        if not scored: return [self.pos]
        scored.sort(key=lambda x:x[0], reverse=True); return [p for _,p in scored]

    def step_commit(self, new_pos: Tuple[int,int], global_map: np.ndarray):
        global global_visited_pois
        if self.env.is_free(new_pos):
            if self.plan and self.plan[0]==new_pos: self.plan.popleft()
            self.pos = new_pos
        self.visited.add(self.pos); self.recent.append(self.pos)
        self.observe_and_share(global_map)
        if self.env.grid[self.pos]==1:
            self.visited_pois.add(self.pos); global_visited_pois.add(self.pos)
            self.local_map[self.pos]=1; global_map[self.pos]=1

# ---------------- init
env = Environment(GRID_SIZE, NUM_OBSTACLES, NUM_POIS)
global_map = np.full((GRID_SIZE, GRID_SIZE), 9, dtype=int)
def random_free_cell(taken:Set[Tuple[int,int]]):
    while True:
        i,j = random.randrange(GRID_SIZE), random.randrange(GRID_SIZE)
        if env.grid[i,j] != -1 and (i,j) not in taken: return (i,j)

robots: List[RoboAgente]=[]; taken=set()
for rid in range(NUM_ROBOTS):
    s = random_free_cell(taken); taken.add(s)
    r = RoboAgente(id=rid, pos=s, env=env); r.visited.add(s); r.recent.append(s)
    r.observe_and_share(global_map); robots.append(r)

def broadcast_merge(robots: List[RoboAgente], global_map: np.ndarray):
    for r in robots:
        mask = (r.local_map != 9); global_map[mask] = r.local_map[mask]
        for p in r.visited_pois: global_visited_pois.add(p)

# ---------------- visual
value_to_index = {-1:0,0:1,1:2,9:3}
cmap = mcolors.ListedColormap(['red','white','green','lightgray'])
norm = mcolors.BoundaryNorm([0,1,2,3,4], cmap.N)
def map_for_display(g): return np.vectorize(lambda v: value_to_index.get(int(v),3))(g)

fig,ax = plt.subplots(figsize=(6,6))
im = ax.imshow(map_for_display(global_map), cmap=cmap, norm=norm)
colors = ["tab:blue","tab:orange","tab:purple","tab:brown","tab:pink"]
scatters=[ax.scatter(r.pos[1], r.pos[0], c=[colors[i%len(colors)]], s=140, edgecolors="black", zorder=5) for i,r in enumerate(robots)]
legend = [Patch(facecolor='red',edgecolor='k',label='Obst'), Patch(facecolor='white',edgecolor='k',label='Livre'),
          Patch(facecolor='green',edgecolor='k',label='POI'), Patch(facecolor='lightgray',edgecolor='k',label='Desconhecido')]
for i in range(NUM_ROBOTS): legend.append(Line2D([0],[0],marker='o',color='w',label=f'Robo {i}',markerfacecolor=colors[i%len(colors)],markersize=8,markeredgecolor='k'))
ax.legend(handles=legend, bbox_to_anchor=(1.02,1.0), loc='upper left'); ax.set_xticks([]); ax.set_yticks([])
ax.set_title("Exploração Multiagente — Mapa Global (descoberto)"); plt.tight_layout()

# ---------------- controle e animação
def resolve_conflicts_and_move(robots, global_map):
    for r in robots: r.observe_and_share(global_map)
    for r in robots:
        if not r.plan:
            path = r.find_nearest_target_path(global_map)
            if path and len(path)>=2: r.plan = deque(path[1:])
    ranked_lists={}; candidates={}
    for r in robots:
        ranked_lists[r.id] = r.ranked_candidates(global_map)
        candidates[r.id] = ranked_lists[r.id][0] if ranked_lists[r.id] else r.pos
    target_to_ids={}
    for rid,target in candidates.items(): target_to_ids.setdefault(target,[]).append(rid)
    assigned={}; occupied=set()
    for target,ids in list(target_to_ids.items()):
        if len(ids)==1: assigned[ids[0]] = target; occupied.add(target)
    for target,ids in list(target_to_ids.items()):
        if len(ids)>1:
            for i,rid in enumerate(sorted(ids)):
                if rid in assigned: continue
                if i==0 and target not in occupied: assigned[rid]=target; occupied.add(target)
                else:
                    chosen=None
                    for cand in ranked_lists[rid]:
                        if cand not in occupied:
                            chosen=cand; break
                    if chosen is None: chosen = [r for r in robots if r.id==rid][0].pos
                    assigned[rid]=chosen; occupied.add(chosen)
    for r in robots:
        if r.id not in assigned: assigned[r.id]=r.pos; occupied.add(r.pos)
    for r in robots: r.step_commit(assigned[r.id], global_map)
    broadcast_merge(robots, global_map)

def update(frame):
    resolve_conflicts_and_move(robots, global_map)
    im.set_data(map_for_display(global_map))
    for i,r in enumerate(robots): scatters[i].set_offsets([[r.pos[1], r.pos[0]]])
    ax.set_title(f"Passo {frame+1}/{STEPS} — Células descobertas: {np.sum(global_map != 9)}/{GRID_SIZE*GRID_SIZE}")
    return [im] + scatters

ani = FuncAnimation(fig, update, frames=STEPS, interval=250, blit=False, repeat=False)
plt.show()

print("\nAmbiente (ground truth):\n", env.grid)
print("\nMapa global construído:\n", global_map)
print("\nPOIs visitados:", sorted(global_visited_pois))
