#!/usr/bin/env python3
"""
Multi-ambulance runtime simulator with dynamic replanning.
- Spawns multiple ambulances at farthest nodes from hospital(s)
- Uses `replanner.dijkstra_weighted` to compute/update routes
- Emits per-ambulance COEs and a combined `lights_runtime_multi.coe` and `runtime_multi_log.txt`
"""
import os
import random
import math
from collections import defaultdict
from replanner import dijkstra_weighted

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(SCRIPT_DIR, 'COE results')

N_AMB = 2
REPLAN_INTERVAL = 5
PREEMPT_RATIO = 0.15

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

def build_graph(nodes, polylines):
    node_map = {nodes[i]: i for i in range(len(nodes))}
    def find_nearest_node_idx(pt):
        if pt in node_map: return node_map[pt]
        best=None; bd=1e9
        for i,p in enumerate(nodes):
            d = (p[0]-pt[0])**2 + (p[1]-pt[1])**2
            if d<bd: bd=d; best=i
        return best
    adj = defaultdict(list)
    for poly in polylines:
        if len(poly) < 2: continue
        a = find_nearest_node_idx(poly[0]); b = find_nearest_node_idx(poly[-1])
        if a is None or b is None or a==b: continue
        w = euclid(nodes[a], nodes[b])
        adj[a].append((b,w)); adj[b].append((a,w))
    return adj

def main():
    nodes = load_nodes(os.path.join(SCRIPT_DIR,'map_data.coe'))
    polylines = load_edges_polylines(os.path.join(SCRIPT_DIR,'map_data_edges.coe'))
    adj = build_graph(nodes, polylines)
    N = len(nodes)

    counts_vals = parse_coe_points(os.path.join(OUT_DIR,'vehicles_counts_full.coe'))
    if len(counts_vals) < N:
        counts_vals += [0]*(N-len(counts_vals))

    # hospitals -> choose first
    hosp = []
    with open(os.path.join(OUT_DIR,'hospitals.txt'),'r',encoding='utf-8') as f:
        for L in f:
            L=L.strip();
            if not L: continue
            x,y = [int(s) for s in L.split(',')]
            hosp.append((x,y))
    # map hospital to nearest node
    node_map = {nodes[i]: i for i in range(N)}
    def nearest_node(pt):
        best=None; bd=1e9
        for i,p in enumerate(nodes):
            d = (p[0]-pt[0])**2 + (p[1]-pt[1])**2
            if d<bd: bd=d; best=i
        return best
    hosp_nodes = [nearest_node(h) for h in hosp]
    target = hosp_nodes[0]

    # choose spawn nodes = farthest N_AMB nodes from target
    # compute dists by Dijkstra simple distance
    import heapq
    dist = [math.inf]*N; dist[target]=0; pq=[(0,target)]
    while pq:
        d,u = heapq.heappop(pq)
        if d!=dist[u]: continue
        for v,w in adj.get(u,[]):
            nd = d + w
            if nd < dist[v]: dist[v]=nd; heapq.heappush(pq,(nd,v))
    nodes_sorted = sorted(range(N), key=lambda i: dist[i] if dist[i]<math.inf else 1e9, reverse=True)
    spawns = nodes_sorted[:N_AMB]

    ambulances = []
    for si in spawns:
        p = dijkstra_weighted(adj, nodes, counts_vals, si, target)
        if p is None: p = [si, target]
        ambulances.append({'start':si, 'path':p, 'pos_idx':0, 'done':False})

    ticks = 0; max_ticks = 1000
    # storage for per-ambulance frames
    amb_frames = [[] for _ in range(N_AMB)]

    while ticks < max_ticks and not all(a['done'] for a in ambulances):
        ticks += 1
        # each ambulance step
        for ai,a in enumerate(ambulances):
            if a['done']: continue
            # if time to replan
            if ticks % REPLAN_INTERVAL == 0 and a['pos_idx'] < len(a['path'])-1:
                newp = dijkstra_weighted(adj, nodes, counts_vals, a['path'][a['pos_idx']], target)
                if newp: a['path'] = newp; a['pos_idx'] = 0
            # move forward one node if possible
            if a['pos_idx'] < len(a['path'])-1:
                a['pos_idx'] += 1
            else:
                a['done'] = True
            cur = a['path'][a['pos_idx']]
            x,y = nodes[cur]
            word = ((x & 0x3FF) << 22) | ((y & 0x3FF) << 12)
            amb_frames[ai].append(word & 0xFFFFFFFF)

        # optionally update counts (simulate leaving junctions)
        # simple: decrement counts at nodes where ambulances just passed
        for a in ambulances:
            idx = a['path'][max(0,a['pos_idx']-1)]
            if counts_vals[idx] > 0: counts_vals[idx] = max(0, counts_vals[idx]-1)

    # write outputs per ambulance
    for ai,frames in enumerate(amb_frames):
        outp = os.path.join(OUT_DIR, f'ambulance_runtime_{ai}.coe')
        with open(outp,'w',encoding='utf-8') as f:
            f.write('memory_initialization_radix=16;\n')
            f.write('memory_initialization_vector=\n')
            for i,w in enumerate(frames):
                f.write(format(w,'08X'))
                f.write(',' if i < len(frames)-1 else ';\n')
        print('WROTE', outp)

    # build lights runtime: start from original lights and apply preempt for first PREEMPT_RATIO of each path
    full_lights = [[0,60] for _ in range(N)]
    orig_lights_path = os.path.join(OUT_DIR,'lights.coe')
    if os.path.exists(orig_lights_path):
        vals = parse_coe_points(orig_lights_path)
        for i,v in enumerate(vals[:N]):
            st = (v>>14)&0x3; tm = v & 0x3FFF
            full_lights[i]=[st,tm]
    for a in ambulances:
        p = a['path']
        pre = max(1, int(len(p)*PREEMPT_RATIO))
        for ni in p[:pre]: full_lights[ni] = [1,60]

    out_l = os.path.join(OUT_DIR,'lights_runtime_multi.coe')
    with open(out_l,'w',encoding='utf-8') as f:
        f.write('memory_initialization_radix=16;\n')
        f.write('memory_initialization_vector=\n')
        for i,(st,tm) in enumerate(full_lights):
            word = ((st & 0x3) << 14) | (tm & 0x3FFF)
            f.write(format(word,'04X'))
            f.write(',' if i < N-1 else ';\n')
    print('WROTE', out_l)

    # runtime log
    with open(os.path.join(OUT_DIR,'runtime_multi_log.txt'),'w',encoding='utf-8') as f:
        f.write(f'ticks={ticks}\n')
        for i,a in enumerate(ambulances):
            f.write(f'amb_{i}_start={a["start"]}\n')
            f.write(f'amb_{i}_path_len={len(a["path"])}\n')
            f.write(f'amb_{i}_done={a["done"]}\n')
    print('WROTE', os.path.join(OUT_DIR,'runtime_multi_log.txt'))

    # produce simple visualization PPM
    try:
        from visualize_runtime import make_ppm_overlay
        make_ppm_overlay(os.path.join(OUT_DIR,'framebuffer_heat.coe'), nodes, [a['path'] for a in ambulances], os.path.join(OUT_DIR,'stage2.ppm'))
        print('WROTE', os.path.join(OUT_DIR,'stage2.ppm'))
    except Exception as e:
        print('visualization skipped:', e)

if __name__ == '__main__':
    main()
