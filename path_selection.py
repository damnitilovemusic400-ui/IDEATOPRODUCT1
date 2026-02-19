#!/usr/bin/env python3
"""
Stage-1: ambulance path selection.
- Reads `map_data.coe`, `map_data_edges.coe`, `vehicles_counts_full.coe`, and `hospitals.txt` from `COE results`.
- Builds a graph (endpoints of edge polylines), computes a traffic-aware shortest path
  from the node farthest from the chosen hospital to the hospital, and emits:
  - `vehicle_ambulance_path.coe` (packed x,y frames)
  - `lights_preempt.coe` (lights with preemption for the first 15% of the route)
  - `route_log.txt` (spawn/hospital/path summary)

Usage: run from the `sumoxml` folder next to `COE results`.
"""
import os
import math
import heapq
from collections import defaultdict

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(SCRIPT_DIR, 'COE results')

# Files
NODES_COE = os.path.join(SCRIPT_DIR, 'map_data.coe')
EDGES_COE = os.path.join(SCRIPT_DIR, 'map_data_edges.coe')
COUNTS_FULL = os.path.join(OUT_DIR, 'vehicles_counts_full.coe')
HOSPITALS = os.path.join(OUT_DIR, 'hospitals.txt')
ORIG_LIGHTS = os.path.join(OUT_DIR, 'lights.coe')

if not os.path.exists(NODES_COE) or not os.path.exists(EDGES_COE):
    raise SystemExit('map_data.coe or map_data_edges.coe missing')
if not os.path.exists(COUNTS_FULL) or not os.path.exists(HOSPITALS):
    raise SystemExit('Counts or hospitals missing in COE results')

# helpers
def parse_coe_points(path):
    with open(path,'r',encoding='utf-8') as f:
        text = f.read()
    for ch in [';', '\n', '\r']:
        text = text.replace(ch, ',')
    toks = [t.strip() for t in text.split(',') if t.strip()]
    vals = []
    for t in toks:
        if t.lower().startswith('memory_initialization'):
            continue
        tt = t
        if tt.lower().startswith('0x'):
            tt = tt[2:]
        if all(c in '0123456789abcdefABCDEF' for c in tt):
            vals.append(int(tt,16))
    return vals

def load_nodes(path):
    vals = parse_coe_points(path)
    nodes = []
    for v in vals:
        x = (v>>10)&0x3FF
        y = v & 0x3FF
        nodes.append((x,y))
    return nodes

def load_edges_polylines(path):
    # edges file is a sequence of 19-bit coords, segments end when bit19 flag set (0x80000)
    vals = parse_coe_points(path)
    polylines = []
    cur = []
    for v in vals:
        flag = (v & 0x80000) != 0
        coord = v & 0x7FFFF
        x = (coord>>10)&0x3FF
        y = coord & 0x3FF
        cur.append((x,y))
        if flag:
            if cur:
                polylines.append(cur)
            cur = []
    if cur:
        polylines.append(cur)
    return polylines

def euclid(a,b):
    return math.hypot(a[0]-b[0], a[1]-b[1])

# Build graph: map each polyline's endpoints to nearest node index
nodes = load_nodes(NODES_COE)
node_map = {nodes[i]: i for i in range(len(nodes))}
polylines = load_edges_polylines(EDGES_COE)

# helper to find nearest node index for a point (exact match preferred)
def find_nearest_node_idx(pt):
    if pt in node_map:
        return node_map[pt]
    # fallback: nearest by euclidean
    best = None; bestd = 1e9
    for i,p in enumerate(nodes):
        d = (p[0]-pt[0])**2 + (p[1]-pt[1])**2
        if d < bestd:
            bestd = d; best = i
    return best

# Build adjacency
adj = defaultdict(list)
for poly in polylines:
    if len(poly) < 2:
        continue
    a = find_nearest_node_idx(poly[0])
    b = find_nearest_node_idx(poly[-1])
    if a is None or b is None or a==b:
        continue
    w = euclid(nodes[a], nodes[b])
    adj[a].append((b,w))
    adj[b].append((a,w))

N = len(nodes)

# load counts full
counts_vals = []
with open(COUNTS_FULL,'r',encoding='utf-8') as f:
    text = f.read()
for ch in [';', '\n', '\r']:
    text = text.replace(ch, ',')
toks = [t.strip() for t in text.split(',') if t.strip()]
for t in toks:
    if t.lower().startswith('memory_initialization'):
        continue
    if all(c in '0123456789abcdefABCDEF' for c in t):
        counts_vals.append(int(t,16))
if len(counts_vals) < N:
    counts_vals += [0]*(N-len(counts_vals))

# load hospitals
hosp = []
with open(HOSPITALS,'r',encoding='utf-8') as f:
    for L in f:
        L=L.strip()
        if not L: continue
        try:
            x,y = [int(s) for s in L.split(',')]
            hosp.append((x,y))
        except:
            continue
if not hosp:
    raise SystemExit('No hospitals found')

# map hospitals to nearest node
hosp_nodes = [find_nearest_node_idx(h) for h in hosp]

# Read hospital metadata (beds, specialty) if available, else generate defaults.
hosp_meta_path = os.path.join(OUT_DIR, 'hospitals_meta.txt')
hosp_meta = []
if os.path.exists(hosp_meta_path):
    with open(hosp_meta_path, 'r', encoding='utf-8') as f:
        for L in f:
            L=L.strip()
            if not L: continue
            parts = [p.strip() for p in L.split(',')]
            if len(parts) >= 4:
                try:
                    x = int(parts[0]); y = int(parts[1]); beds = int(parts[2]); spec = float(parts[3])
                    hosp_meta.append({'coord':(x,y),'beds':beds,'specialty':spec})
                except:
                    continue

# If no meta file, generate reasonable defaults and write the file for transparency
if not hosp_meta:
    import random
    hosp_meta = []
    for (hx,hy) in hosp:
        beds = random.randint(50,300)
        # specialty: 1.0 means has required emergency surgery, 0.0 means not
        spec = float(random.choice([0,1]))
        hosp_meta.append({'coord':(hx,hy),'beds':beds,'specialty':spec})
    # ensure at least one hospital has specialty
    if not any(h['specialty']>0 for h in hosp_meta):
        hosp_meta[0]['specialty'] = 1.0
    # write out
    with open(hosp_meta_path, 'w', encoding='utf-8') as f:
        for h in hosp_meta:
            f.write(f"{h['coord'][0]},{h['coord'][1]},{h['beds']},{h['specialty']}\n")

# map meta entries to nodes
hosp_nodes = [find_nearest_node_idx(h['coord']) for h in hosp_meta]
# compute spawn node as the node farthest from any hospital
def dijkstra(start):
    dist = [math.inf]*N
    prev = [-1]*N
    dist[start]=0
    pq=[(0,start)]
    while pq:
        d,u = heapq.heappop(pq)
        if d!=dist[u]: continue
        for v,w in adj.get(u,[]):
            nd = d + w
            if nd < dist[v]:
                dist[v]=nd; prev[v]=u
                heapq.heappush(pq,(nd,v))
    return dist,prev

# distances from each hospital
dist_from_each = []
for node_idx in hosp_nodes:
    if node_idx is None:
        dist_from_each.append([math.inf]*N)
    else:
        d,_ = dijkstra(node_idx)
        dist_from_each.append(d)

# for each node, distance to nearest hospital
min_dist_to_hosp = [min(dist_from_each[h][i] for h in range(len(dist_from_each))) for i in range(N)]
spawn_idx = max(range(N), key=lambda i: min_dist_to_hosp[i] if min_dist_to_hosp[i] < math.inf else -1)

# weighted dijkstra for path cost (uses counts_vals)
def dijkstra_weighted(start, target, alpha=1.0, beta=2.0):
    dist = [math.inf]*N
    prev = [-1]*N
    dist[start]=0
    pq=[(0,start)]
    maxc = max(1.0, max(counts_vals))
    while pq:
        d,u = heapq.heappop(pq)
        if d!=dist[u]: continue
        if u==target: break
        for v,w in adj.get(u,[]):
            traffic = (counts_vals[u] + counts_vals[v]) / 2.0
            nd = d + alpha*w + beta*(traffic/ maxc)
            if nd < dist[v]:
                dist[v]=nd; prev[v]=u
                heapq.heappush(pq,(nd,v))
    return dist, prev

# choose hospital via cost: distance+traffic (computed) + hospital penalty (beds/specialty)
alpha = 1.0
beta = 2.0
gamma = 5.0
max_beds = max(h['beds'] for h in hosp_meta) if hosp_meta else 1

best_hosp_idx = None
best_cost = float('inf')
best_path = None
for hi, node_idx in enumerate(hosp_nodes):
    if node_idx is None:
        continue
    dist_arr, prev_arr = dijkstra_weighted(spawn_idx, node_idx, alpha, beta)
    dcost = dist_arr[node_idx]
    # hospital penalty: prefer specialty and more beds
    beds = hosp_meta[hi]['beds']
    spec = hosp_meta[hi]['specialty']
    beds_pen = (1.0 - (beds / max_beds))
    spec_pen = (0.0 if spec >= 1.0 else 1.0)
    hosp_penalty = gamma * (beds_pen + spec_pen)
    total_cost = dcost + hosp_penalty
    if total_cost < best_cost:
        best_cost = total_cost
        best_hosp_idx = hi
        # reconstruct path using prev_arr
        path_tmp = []
        u = node_idx
        while u != -1:
            path_tmp.append(u)
            if u == spawn_idx: break
            u = prev_arr[u]
        best_path = list(reversed(path_tmp))

if best_hosp_idx is None:
    raise SystemExit('No reachable hospital chosen')

target_hosp_idx = hosp_nodes[best_hosp_idx]
target_coord = nodes[target_hosp_idx]

# Use the best_path we just computed
path = best_path


# Dijkstra from hospital to find farthest reachable node
def dijkstra(start):
    dist = [math.inf]*N
    prev = [-1]*N
    dist[start]=0
    pq=[(0,start)]
    while pq:
        d,u = heapq.heappop(pq)
        if d!=dist[u]: continue
        for v,w in adj.get(u,[]):
            nd = d + w
            if nd < dist[v]:
                dist[v]=nd; prev[v]=u
                heapq.heappush(pq,(nd,v))
    return dist,prev

dist_from_h, _ = dijkstra(target_hosp_idx)
farthest = max(range(N), key=lambda i: dist_from_h[i] if dist_from_h[i]<math.inf else -1)
spawn_idx = farthest

# if best_path wasn't set earlier (fallback), compute a weighted path
if not path:
    def dijkstra_weighted(start, target):
        dist = [math.inf]*N
        prev = [-1]*N
        dist[start]=0
        pq=[(0,start)]
        maxc = max(1.0, max(counts_vals))
        while pq:
            d,u = heapq.heappop(pq)
            if d!=dist[u]: continue
            if u==target: break
            for v,w in adj.get(u,[]):
                traffic = (counts_vals[u] + counts_vals[v]) / 2.0
                nd = d + alpha*w + beta*(traffic/ maxc)
                if nd < dist[v]:
                    dist[v]=nd; prev[v]=u
                    heapq.heappush(pq,(nd,v))
        return dist, prev

    dist_sp, prev_sp = dijkstra_weighted(spawn_idx, target_hosp_idx)
    if math.isinf(dist_sp[target_hosp_idx]):
        raise SystemExit('No path found from spawn to hospital')

    # reconstruct path
    path = []
    u = target_hosp_idx
    while u!=-1:
        path.append(u)
        if u==spawn_idx: break
        u = prev_sp[u]
    path = list(reversed(path))

# write vehicle_ambulance_path.coe with packed frames (x:10 at bits31..22, y:10 at 21..12)
out_path = os.path.join(OUT_DIR, 'vehicle_ambulance_path.coe')
with open(out_path,'w',encoding='utf-8') as f:
    f.write('memory_initialization_radix=16;\n')
    f.write('memory_initialization_vector=\n')
    for i,ni in enumerate(path):
        x,y = nodes[ni]
        word = ((x & 0x3FF) << 22) | ((y & 0x3FF) << 12)
        f.write(format(word & 0xFFFFFFFF, '08X'))
        f.write(',' if i < len(path)-1 else ';\n')
print('WROTE', out_path)

# create lights_preempt.coe: copy orig lights and override first 15% of path nodes to GREEN (state=1)
full_lights = []
if os.path.exists(ORIG_LIGHTS):
    vals = parse_coe_points(ORIG_LIGHTS)
    for v in vals:
        st = (v>>14)&0x3
        tm = v & 0x3FFF
        full_lights.append([st,tm])
else:
    # default all RED
    full_lights = [[0,60] for _ in range(N)]
if len(full_lights) < N:
    full_lights += [[0,60] for _ in range(N-len(full_lights))]

preempt_count = max(1, int(len(path)*0.15))
for ni in path[:preempt_count]:
    full_lights[ni] = [1, 60]  # GREEN with timer 60

out_lpre = os.path.join(OUT_DIR, 'lights_preempt.coe')
with open(out_lpre,'w',encoding='utf-8') as f:
    f.write('memory_initialization_radix=16;\n')
    f.write('memory_initialization_vector=\n')
    for i,(st,tm) in enumerate(full_lights):
        word = ((st & 0x3) << 14) | (tm & 0x3FFF)
        f.write(format(word,'04X'))
        f.write(',' if i < len(full_lights)-1 else ';\n')
print('WROTE', out_lpre)

# write route log
logp = os.path.join(OUT_DIR, 'route_log.txt')
dist_sp, prev_sp = dijkstra_weighted(spawn_idx, target_hosp_idx, alpha, beta)
with open(logp,'w',encoding='utf-8') as f:
    f.write(f'spawn_node_index={spawn_idx}\n')
    f.write(f'spawn_coord={nodes[spawn_idx]}\n')
    f.write(f'target_hospital_index={target_hosp_idx}\n')
    f.write(f'target_coord={target_coord}\n')
    f.write(f'path_len_nodes={len(path)}\n')
    f.write(f'path_nodes={path}\n')
    f.write(f'path_coords={[nodes[i] for i in path]}\n')
    f.write(f'weighted_cost={dist_sp[target_hosp_idx]}\n')
    # record chosen hospital metadata
    hmeta = hosp_meta[best_hosp_idx]
    f.write(f'hospital_meta_beds={hmeta["beds"]}\n')
    f.write(f'hospital_meta_specialty={hmeta["specialty"]}\n')

print('Stage-1: path and lights preemption written.')
