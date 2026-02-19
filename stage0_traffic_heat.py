#!/usr/bin/env python3
"""
Stage-0 traffic heat simulator.
- Creates per-junction vehicle counters (total 5000, >=50 per active junction)
- Simulates simple arrival/departure and light cycles for a default loop
- Produces outputs in `COE results`:
  - `vehicles_counts.coe` (16-bit counts per active junction)
  - `lights.coe` (updated 16-bit state+timer per junction)
  - `framebuffer_heat.coe` (640x480 RGB565 framebuffer with heat overlays and 2 hospitals)

Usage: run next to existing COE results folder created earlier.
"""
import os
import random
import math
from collections import defaultdict

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(SCRIPT_DIR, 'COE results')
os.makedirs(OUT_DIR, exist_ok=True)

# Parameters
TOTAL_VEH = 5000
MIN_PER_JUNC = 50
SIM_TICKS = 300  # simulation ticks
TICKS_PER_SEC = 10
CYCLE_SECONDS = 10.0  # full traffic light cycle scaled to 10s
# Phase proportions (green, yellow, red) summing to 1.0
PHASE_PROPS = (0.6, 0.1, 0.3)  # green 60%, yellow 10%, red 30%

WIDTH = 640
HEIGHT = 480

# Color map for congestion: list of RGB tuples from low->high
COLORMAP = [
    (0,0,0),        # 0 vehicles -> black
    (255,200,235),  # light pink
    (220,120,200),  # dark pink
    (160,60,170),   # violet
    (90,10,120)     # dark violet (max)
]

# helper: convert rgb888 to rgb565

def rgb565(r,g,b):
    r5 = (r*31)//255
    g6 = (g*63)//255
    b5 = (b*31)//255
    return (r5<<11) | (g6<<5) | b5

# Read nodes
NODES_COE = os.path.join(SCRIPT_DIR, 'map_data.coe')
if not os.path.exists(NODES_COE):
    NODES_COE = os.path.join(os.path.dirname(SCRIPT_DIR), 'map_data.coe')

nodes = []
if os.path.exists(NODES_COE):
    with open(NODES_COE, 'r', encoding='utf-8') as f:
        text = f.read()
    for ch in [';', '\n', '\r']:
        text = text.replace(ch, ',')
    tokens = [t.strip() for t in text.split(',') if t.strip()]
    for t in tokens:
        if t.lower().startswith('memory_initialization'):
            continue
        tt = t
        if tt.lower().startswith('0x'):
            tt = tt[2:]
        if all(c in '0123456789abcdefABCDEF' for c in tt):
            v = int(tt,16)
            x = (v>>10)&0x3FF
            y = v & 0x3FF
            nodes.append((x,y))
else:
    raise SystemExit('map_data.coe not found')

N_TOTAL_NODES = len(nodes)
print('Found', N_TOTAL_NODES, 'nodes')

# Choose active junctions count K such that each has at least MIN_PER_JUNC
K = min(N_TOTAL_NODES, TOTAL_VEH // MIN_PER_JUNC)
if K <= 0:
    K = min(N_TOTAL_NODES, 1)
print('Active junctions:', K)

# pick K indices spread across node list (random sample)
active_idxs = sorted(random.sample(range(N_TOTAL_NODES), K))
active_coords = [nodes[i] for i in active_idxs]

# initialize vehicle counts: distribute TOTAL_VEH across K with at least MIN_PER_JUNC
base = TOTAL_VEH // K
counts = [max(MIN_PER_JUNC, base) for _ in range(K)]
# adjust to match TOTAL_VEH
cur_sum = sum(counts)
if cur_sum > TOTAL_VEH:
    # reduce proportionally
    scale = TOTAL_VEH / cur_sum
    counts = [max(MIN_PER_JUNC, int(c*scale)) for c in counts]
    cur_sum = sum(counts)
# distribute remainder
rem = TOTAL_VEH - cur_sum
i = 0
while rem > 0:
    counts[i%K] += 1
    rem -= 1
    i += 1

# lights initial states and timers per active junction
phase_ticks = [int(CYCLE_SECONDS * TICKS_PER_SEC * p) for p in PHASE_PROPS]
G_T, Y_T, R_T = phase_ticks

lights = []
for i in range(K):
    # random phase offset
    st = random.choice([0,1,2])  # 0=RED,1=GREEN,2=YELLOW
    if st==1:
        t = random.randint(1, G_T)
    elif st==2:
        t = random.randint(1, Y_T)
    else:
        t = random.randint(1, R_T)
    lights.append([st, t])

# Simulation rules
ARRIVAL_LAMBDA = 0.5  # avg arrivals per tick
DEPART_WHEN_GREEN = 2  # vehicles departing per tick when green

# Simple simulation loop
for tick in range(SIM_TICKS):
    # update each junction
    for idx in range(K):
        st, timer = lights[idx]
        # arrivals: Poisson approx by Bernoulli for small lambda
        if random.random() < ARRIVAL_LAMBDA:
            counts[idx] += 1
        # departures
        if st == 1 and counts[idx] > 0:
            d = min(counts[idx], DEPART_WHEN_GREEN)
            counts[idx] -= d
        # decrement timer
        timer -= 1
        if timer <= 0:
            # advance state
            if st == 1:
                st = 2  # green->yellow
                timer = Y_T
            elif st == 2:
                st = 0  # yellow->red
                timer = R_T
            else:
                st = 1  # red->green
                timer = G_T
        lights[idx][0] = st
        lights[idx][1] = timer

# Write vehicles_counts.coe
vc_path = os.path.join(OUT_DIR, 'vehicles_counts.coe')
with open(vc_path, 'w', encoding='utf-8') as f:
    f.write('memory_initialization_radix=16;\n')
    f.write('memory_initialization_vector=\n')
    for i,c in enumerate(counts):
        f.write(format(c & 0xFFFF, '04X'))
        f.write(',' if i < len(counts)-1 else ';\n')
print('WROTE', vc_path)

# Also write a full-length vehicles_counts_full.coe aligned to all nodes (zeros except active)
vc_full_path = os.path.join(OUT_DIR, 'vehicles_counts_full.coe')
with open(vc_full_path, 'w', encoding='utf-8') as f:
    f.write('memory_initialization_radix=16;\n')
    f.write('memory_initialization_vector=\n')
    for ni in range(N_TOTAL_NODES):
        if ni in active_idxs:
            ai = active_idxs.index(ni)
            val = counts[ai]
        else:
            val = 0
        f.write(format(val & 0xFFFF, '04X'))
        f.write(',' if ni < N_TOTAL_NODES-1 else ';\n')
print('WROTE', vc_full_path)

# Write active junctions mapping file (node_index,count)
map_path = os.path.join(OUT_DIR, 'vehicles_counts_map.txt')
with open(map_path, 'w', encoding='utf-8') as f:
    for ai, ni in enumerate(active_idxs):
        f.write(f'{ni},{counts[ai]}\n')
print('WROTE', map_path)

# Update lights.coe in OUT_DIR for active junctions (we'll create full-length lights list matching earlier lights.coe length by defaulting other nodes to RED 0 timer)
# Load existing lights.coe if exists to get length
orig_lights_path = os.path.join(OUT_DIR, 'lights.coe')
full_len = N_TOTAL_NODES
full_lights = [[0, R_T] for _ in range(full_len)]
# If existing lights.coe present, read and use as base
if os.path.exists(orig_lights_path):
    with open(orig_lights_path, 'r', encoding='utf-8') as f:
        text = f.read()
    for ch in [';', '\n', '\r']:
        text = text.replace(ch, ',')
    tokens = [t.strip() for t in text.split(',') if t.strip()]
    vals = []
    for t in tokens:
        if t.lower().startswith('memory_initialization'):
            continue
        if all(c in '0123456789abcdefABCDEF' for c in t):
            vals.append(int(t,16))
    for i,v in enumerate(vals[:full_len]):
        st = (v>>14)&0x3
        tm = v & 0x3FFF
        full_lights[i] = [st, tm]
# overwrite active entries
for ai,li in enumerate(active_idxs):
    full_lights[li] = lights[ai][:]
# write new lights.coe
out_lights_path = os.path.join(OUT_DIR, 'lights.coe')
with open(out_lights_path, 'w', encoding='utf-8') as f:
    f.write('memory_initialization_radix=16;\n')
    f.write('memory_initialization_vector=\n')
    for i,(st,timer) in enumerate(full_lights):
        word = ((st & 0x3) << 14) | (timer & 0x3FFF)
        f.write(format(word,'04X'))
        f.write(',' if i < len(full_lights)-1 else ';\n')
print('WROTE', out_lights_path)

# Produce framebuffer_heat.coe by overlaying heat rectangles onto existing framebuffer (if exists), else blank
fb_src = os.path.join(OUT_DIR, 'framebuffer.coe')
fb = [rgb565(0,0,0)] * (WIDTH*HEIGHT)
if os.path.exists(fb_src):
    with open(fb_src,'r',encoding='utf-8') as f:
        text = f.read()
    for ch in [';', '\n', '\r']:
        text = text.replace(ch, ',')
    tokens = [t.strip() for t in text.split(',') if t.strip()]
    vals = [int(t,16) for t in tokens if all(c in '0123456789abcdefABCDEF' for c in t)]
    for i,v in enumerate(vals[:WIDTH*HEIGHT]):
        fb[i] = v & 0xFFFF
else:
    # keep black background
    pass

# compute max count for normalization
maxc = max(counts) if counts else 1

# For each active junction draw a small 7x3 horizontal rectangle to its right (or left if near edge)
for (cx,cy),cnt in zip(active_coords, counts):
    # choose rectangle size proportional to count
    norm = cnt / maxc
    # pick color index from colormap
    idx = int(norm * (len(COLORMAP)-1))
    r,g,b = COLORMAP[idx]
    color = rgb565(r,g,b)
    w = 7
    h = 3
    # place to the right unless near border
    x0 = cx + 3
    if x0 + w >= WIDTH:
        x0 = cx - 3 - w
    y0 = cy - h//2
    for yy in range(y0, y0+h):
        for xx in range(x0, x0+w):
            if 0<=xx<WIDTH and 0<=yy<HEIGHT:
                fb[yy*WIDTH + xx] = color

# Place 2 hospitals as black 3x3 blocks at random non-road places (we'll pick positions away from nodes > 20 pixels)
hospitals = []
tries = 0
while len(hospitals) < 2 and tries < 2000:
    tries += 1
    x = random.randint(10, WIDTH-11)
    y = random.randint(10, HEIGHT-11)
    ok = True
    for nx,ny in nodes:
        if abs(nx - x) < 20 and abs(ny - y) < 20:
            ok = False; break
    if ok:
        hospitals.append((x,y))

for (hx,hy) in hospitals:
    for yy in range(hy-1, hy+2):
        for xx in range(hx-1, hx+2):
            if 0<=xx<WIDTH and 0<=yy<HEIGHT:
                fb[yy*WIDTH + xx] = rgb565(0,0,0)

# write framebuffer_heat.coe
fb_out = os.path.join(OUT_DIR, 'framebuffer_heat.coe')
with open(fb_out,'w',encoding='utf-8') as f:
    f.write('memory_initialization_radix=16;\n')
    f.write('memory_initialization_vector=\n')
    total = WIDTH*HEIGHT
    for i,val in enumerate(fb):
        f.write(format(val & 0xFFFF, '04X'))
        f.write(',' if i < total-1 else ';\n')
print('WROTE', fb_out)

# write hospitals list
hosp_path = os.path.join(OUT_DIR, 'hospitals.txt')
with open(hosp_path, 'w', encoding='utf-8') as f:
    for (hx,hy) in hospitals:
        f.write(f'{hx},{hy}\n')
print('WROTE', hosp_path)

print('\nStage-0 complete. Active junctions:', K, 'Total vehicles:', sum(counts))
