#!/usr/bin/env python3
"""
Stage-2 full rules simulator.
- Applies tiered preemption (15%/40%/45% with minima) along ambulance path
- First 15%: immediate green double time, red quarter time; others in junction stay red until pass
- Next 40%: green 1.5x, red 0.5x; others get 0.5x green
- Next 45%: green normal, red 0.6x
- After ambulance passes a junction, swap timings for one cycle between selected approach and others
- Emits: `lights_stage2.coe`, `framebuffer_stage2.coe`, `runtime_stage2_log.txt`

Assumptions:
- Each node has a single stored light entry representing the ambulance path approach.
- "Others" timings are simulated separately and affect behavior only in the simulator; only the selected approach value is written out to COE.
"""
import os, math, random
from collections import defaultdict

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(SCRIPT_DIR, 'COE results')

NODES_COE = os.path.join(SCRIPT_DIR, 'map_data.coe')
PATH_LOG = os.path.join(OUT_DIR, 'route_log.txt')
COUNTS_FULL = os.path.join(OUT_DIR, 'vehicles_counts_full.coe')
ORIG_LIGHTS = os.path.join(OUT_DIR, 'lights.coe')
FB_HEAT = os.path.join(OUT_DIR, 'framebuffer_heat.coe')

if not os.path.exists(PATH_LOG):
    raise SystemExit('route_log.txt not found; run path_selection first')

# load nodes
def parse_coe(path):
    with open(path,'r',encoding='utf-8') as f:
        t = f.read()
    for ch in [';', '\n', '\r']:
        t = t.replace(ch, ',')
    toks = [s.strip() for s in t.split(',') if s.strip()]
    vals = []
    for s in toks:
        if s.lower().startswith('memory_initialization'): continue
        tt = s
        if tt.lower().startswith('0x'): tt = tt[2:]
        if all(c in '0123456789abcdefABCDEF' for c in tt):
            vals.append(int(tt,16))
    return vals

nodes_raw = parse_coe(NODES_COE)
nodes = [( (v>>10)&0x3FF, v & 0x3FF) for v in nodes_raw]
N = len(nodes)

# read path_log
path_nodes = None
with open(PATH_LOG,'r',encoding='utf-8') as f:
    for L in f:
        if L.startswith('path_nodes='):
            rhs = L.split('=',1)[1].strip()
            rhs = rhs.strip()
            if rhs.startswith('[') and rhs.endswith(']'):
                inner = rhs[1:-1].strip()
                if inner:
                    path_nodes = [int(p.strip()) for p in inner.split(',')]
                else:
                    path_nodes = []
            break
if path_nodes is None:
    raise SystemExit('path_nodes not found in route_log.txt')
L = len(path_nodes)
if L==0:
    raise SystemExit('Empty path')

# load counts
counts_vals = parse_coe(COUNTS_FULL)
if len(counts_vals) < N:
    counts_vals += [0]*(N-len(counts_vals))

# base timings
TICKS_PER_SEC = 10
CYCLE_SECONDS = 10.0
PHASE_PROPS = (0.6, 0.1, 0.3)  # green,yellow,red proportions
G_T = int(CYCLE_SECONDS * TICKS_PER_SEC * PHASE_PROPS[0])
Y_T = int(CYCLE_SECONDS * TICKS_PER_SEC * PHASE_PROPS[1])
R_T = int(CYCLE_SECONDS * TICKS_PER_SEC * PHASE_PROPS[2])

# determine tier counts with minima
def partition_counts(L):
    n1 = max(1, int(round(0.15 * L)))
    n2 = max(3, int(round(0.40 * L)))
    n3 = L - n1 - n2
    if n3 < 4:
        # ensure minimums by shifting
        need = 4 - n3
        take = min(need, n2 - 3)
        n2 -= take; n3 += take
    if n3 < 0:
        # fallback evenly
        n1 = max(1, int(L*0.15))
        n2 = max(3, int(L*0.40))
        n3 = L - n1 - n2
    return n1,n2,n3

n1,n2,n3 = partition_counts(L)

# compute per-node modifiers
tiers = {}
for i,ni in enumerate(path_nodes):
    if i < n1:
        tiers[ni] = 'tier1'
    elif i < n1 + n2:
        tiers[ni] = 'tier2'
    else:
        tiers[ni] = 'tier3'

# load original lights as base
orig_vals = parse_coe(ORIG_LIGHTS)
full_lights = [[0, R_T] for _ in range(N)]
for i,v in enumerate(orig_vals[:N]):
    st = (v>>14)&0x3; tm = v & 0x3FFF
    full_lights[i] = [st, tm]

# selected approach timings and others timings
selected_timings = {}
others_timings = {}
for ni in range(N):
    selected_timings[ni] = {'G':G_T,'Y':Y_T,'R':R_T}
    others_timings[ni] = {'G':G_T,'Y':Y_T,'R':R_T}

# apply modifiers per tier
for i,ni in enumerate(path_nodes):
    tier = tiers[ni]
    if tier=='tier1':
        selected_timings[ni]['G'] = max(1, int(G_T * 2.0))
        selected_timings[ni]['R'] = max(1, int(R_T * 0.25))
        others_timings[ni]['G'] = 0  # others remain red until passed
    elif tier=='tier2':
        selected_timings[ni]['G'] = max(1, int(G_T * 1.5))
        selected_timings[ni]['R'] = max(1, int(R_T * 0.5))
        others_timings[ni]['G'] = max(0, int(G_T * 0.5))
    else:
        selected_timings[ni]['G'] = G_T
        selected_timings[ni]['R'] = max(1, int(R_T * 0.6))
        others_timings[ni]['G'] = G_T

# simulation: ambulance moves 1 node per STEP_TICKS (simplify: 1 tick per node)
STEP_TICKS = 1

# create runtime copies
sim_selected_state = {i: [0, selected_timings[i]['R']] for i in range(N)}
sim_others_state = {i: [0, others_timings[i]['R']] for i in range(N)}

# immediate preemption at start: first 15% nodes turn green instantly
for idx in path_nodes[:n1]:
    sim_selected_state[idx] = [1, selected_timings[idx]['G']]

# record ambulance frames and lights over time
amb_frames = []
swap_schedule = {}  # node -> ticks remaining for swapped cycle

tick = 0
for step, cur in enumerate(path_nodes):
    # ambulance is at cur for STEP_TICKS ticks
    for t in range(STEP_TICKS):
        # decrement timers
        for i in range(N):
            if sim_selected_state[i][1] > 0:
                sim_selected_state[i][1] = max(0, sim_selected_state[i][1]-1)
            if sim_others_state[i][1] > 0:
                sim_others_state[i][1] = max(0, sim_others_state[i][1]-1)
        # record frame (store selected approach word)
        x,y = nodes[cur]
        word = ((x & 0x3FF) << 22) | ((y & 0x3FF) << 12)
        amb_frames.append(word)
        tick += 1

    # when ambulance passes cur, schedule post-pass swap for one full cycle (use base cycle length in ticks)
    swap_len = G_T + Y_T + R_T
    # swap selected and others timings for this node for swap_len ticks
    sel = selected_timings[cur]
    oth = others_timings[cur]
    # set selected to others timing, and others to selected, for swap duration
    sim_selected_state[cur] = [1, oth['G']]
    sim_others_state[cur] = [0, sel['G']]
    swap_schedule[cur] = swap_len

    # decrement swap schedules while moving to next node: advance time by 1 tick to simulate passage
    # reduce all swap timers by STEP_TICKS and revert when expired
    for i in list(swap_schedule.keys()):
        swap_schedule[i] -= STEP_TICKS
        if swap_schedule[i] <= 0:
            # revert to normal selected timings
            sim_selected_state[i] = [0, selected_timings[i]['R']]
            sim_others_state[i] = [0, others_timings[i]['R']]
            del swap_schedule[i]

# write ambulance_runtime_stage2.coe
out_amb = os.path.join(OUT_DIR, 'ambulance_runtime_stage2.coe')
with open(out_amb,'w',encoding='utf-8') as f:
    f.write('memory_initialization_radix=16;\n')
    f.write('memory_initialization_vector=\n')
    for i,w in enumerate(amb_frames):
        f.write(format(w & 0xFFFFFFFF,'08X'))
        f.write(',' if i < len(amb_frames)-1 else ';\n')

# write lights_stage2.coe using sim_selected_state current values
out_l = os.path.join(OUT_DIR, 'lights_stage2.coe')
with open(out_l,'w',encoding='utf-8') as f:
    f.write('memory_initialization_radix=16;\n')
    f.write('memory_initialization_vector=\n')
    for i in range(N):
        st,tm = sim_selected_state[i]
        word = ((st & 0x3) << 14) | (tm & 0x3FFF)
        f.write(format(word,'04X'))
        f.write(',' if i < N-1 else ';\n')

# produce framebuffer_stage2 by overlaying heat and path and hospitals
fb_pixels = []
if os.path.exists(FB_HEAT):
    # reuse earlier framebuffer heat reading routine: simple parse
    vals = parse_coe(FB_HEAT)
    fb_pixels = [v & 0xFFFF for v in vals]
else:
    fb_pixels = [0] * (640*480)

# draw path as bright red dots
for ni in path_nodes:
    x,y = nodes[ni]
    for dx in (-1,0,1):
        for dy in (-1,0,1):
            xx = x+dx; yy = y+dy
            if 0<=xx<640 and 0<=yy<480:
                fb_pixels[yy*640 + xx] = ((31<<11) | (0<<5) | 0)  # red

out_fb = os.path.join(OUT_DIR, 'framebuffer_stage2.coe')
with open(out_fb,'w',encoding='utf-8') as f:
    f.write('memory_initialization_radix=16;\n')
    f.write('memory_initialization_vector=\n')
    total = 640*480
    for i,val in enumerate(fb_pixels[:total]):
        f.write(format(val & 0xFFFF, '04X'))
        f.write(',' if i < total-1 else ';\n')

# runtime log
with open(os.path.join(OUT_DIR,'runtime_stage2_log.txt'),'w',encoding='utf-8') as f:
    f.write(f'path_len={L}\n')
    f.write(f'n1={n1},n2={n2},n3={n3}\n')
    f.write(f'path_nodes={path_nodes}\n')

print('WROTE', out_amb, out_l, out_fb)
