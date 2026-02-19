#!/usr/bin/env python3
"""
Produce FPGA-ready COEs from existing geometry COEs.
- Reads `map_data.coe` and `map_data_edges.coe` (searches nearby locations)
- Writes into `COE results/`: `framebuffer.coe` (RGB565 640x480), `geometry32.coe` (x10,y10,typ4,color8 packed), `vehicles_template.coe` (editable)
- Inverts Y so SUMO/world Y-up maps to screen Y-down.
"""
import os
import sys

WIDTH = 640
HEIGHT = 480

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(SCRIPT_DIR, 'COE results')
os.makedirs(OUT_DIR, exist_ok=True)

# Candidate input filenames (look in script dir and parent)
CANDIDATES = [
    os.path.join(SCRIPT_DIR, 'map_data.coe'),
    os.path.join(SCRIPT_DIR, 'map_data_edges.coe'),
    os.path.join(os.path.dirname(SCRIPT_DIR), 'map_data.coe'),
    os.path.join(os.path.dirname(SCRIPT_DIR), 'map_data_edges.coe'),
]

NODES_COE = None
EDGES_COE = None
for p in CANDIDATES:
    if os.path.basename(p).lower() == 'map_data.coe' and os.path.exists(p):
        NODES_COE = p
    if os.path.basename(p).lower() == 'map_data_edges.coe' and os.path.exists(p):
        EDGES_COE = p

# also allow if user placed files directly under SCRIPT_DIR
if NODES_COE is None and os.path.exists(os.path.join(SCRIPT_DIR, 'map_data.coe')):
    NODES_COE = os.path.join(SCRIPT_DIR, 'map_data.coe')
if EDGES_COE is None and os.path.exists(os.path.join(SCRIPT_DIR, 'map_data_edges.coe')):
    EDGES_COE = os.path.join(SCRIPT_DIR, 'map_data_edges.coe')

# If none found attempt to search workspace (script dir subtree)
if NODES_COE is None or EDGES_COE is None:
    for root, dirs, files in os.walk(SCRIPT_DIR):
        for fn in files:
            if fn.lower() == 'map_data.coe' and NODES_COE is None:
                NODES_COE = os.path.join(root, fn)
            if fn.lower() == 'map_data_edges.coe' and EDGES_COE is None:
                EDGES_COE = os.path.join(root, fn)

print('Using nodes:', NODES_COE)
print('Using edges:', EDGES_COE)

# Helpers

def rgb888_to_rgb565(r, g, b):
    r5 = (r * 31) // 255
    g6 = (g * 63) // 255
    b5 = (b * 31) // 255
    return (r5 << 11) | (g6 << 5) | b5


def read_coe_points(path):
    if not path or not os.path.exists(path):
        return []
    with open(path, 'r', encoding='utf-8') as f:
        text = f.read()
    for ch in [';', '\n', '\r']:
        text = text.replace(ch, ',')
    tokens = [t.strip() for t in text.split(',') if t.strip()]
    vals = []
    for t in tokens:
        if t.lower().startswith('memory_initialization'):
            continue
        tt = t
        if tt.lower().startswith('0x'):
            tt = tt[2:]
        if all(c in '0123456789abcdefABCDEF' for c in tt):
            try:
                v = int(tt, 16)
            except Exception:
                continue
            x = (v >> 10) & 0x3FF
            y = v & 0x3FF
            vals.append((x, y))
    return vals

# Read inputs
nodes = read_coe_points(NODES_COE)
edges = read_coe_points(EDGES_COE)
print('Read', len(nodes), 'nodes and', len(edges), 'edge points')

# Prepare framebuffer
BG_RGB = (0, 0, 0)
ROAD_RGB = (200, 200, 200)
JUNCTION_RGB = (255, 0, 0)

bg = rgb888_to_rgb565(*BG_RGB)
road = rgb888_to_rgb565(*ROAD_RGB)
junction = rgb888_to_rgb565(*JUNCTION_RGB)

fb = [bg] * (WIDTH * HEIGHT)

# draw primitives (small, CPU rasterizer for offline generation)

def set_pixel(buf, x, y, color):
    if 0 <= x < WIDTH and 0 <= y < HEIGHT:
        buf[y * WIDTH + x] = color


def draw_line(buf, x0, y0, x1, y1, color):
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        set_pixel(buf, x0, y0, color)
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


def draw_filled_circle(buf, cx, cy, r, color):
    for dy in range(-r, r+1):
        y = cy + dy
        span = int((r*r - dy*dy) ** 0.5)
        for dx in range(-span, span+1):
            x = cx + dx
            set_pixel(buf, x, y, color)

# Draw edges (pairs) with Y inversion
for i in range(0, len(edges) - 1, 2):
    x0, y0 = edges[i]
    x1, y1 = edges[i+1]
    x0 = max(0, min(WIDTH-1, x0))
    x1 = max(0, min(WIDTH-1, x1))
    y0 = max(0, min(HEIGHT-1, y0))
    y1 = max(0, min(HEIGHT-1, y1))
    y0 = HEIGHT - 1 - y0
    y1 = HEIGHT - 1 - y1
    draw_line(fb, x0, y0, x1, y1, road)

# Draw nodes as small circles
for (x, y) in nodes:
    if 0 <= x < WIDTH and 0 <= y < HEIGHT:
        yi = HEIGHT - 1 - y
        draw_filled_circle(fb, x, yi, 2, junction)

# Write framebuffer COE
fb_path = os.path.join(OUT_DIR, 'framebuffer.coe')
with open(fb_path, 'w', encoding='utf-8') as f:
    f.write('memory_initialization_radix=16;\n')
    f.write('memory_initialization_vector=\n')
    total = WIDTH * HEIGHT
    for i, val in enumerate(fb):
        token = format(val & 0xFFFF, '04X')
        end = ',' if i < total-1 else ';'
        f.write(token + end + '\n')
print('WROTE', fb_path, 'pixels=', WIDTH*HEIGHT)

# Emit geometry32.coe: x:10 y:10 typ:4 color:8
GEOM_PATH = os.path.join(OUT_DIR, 'geometry32.coe')
ROAD_COLOR8 = 0xC8
NODE_COLOR8 = 0xF0

def pack_geom(x, y, typ, color8):
    return ((x & 0x3FF) << 22) | ((y & 0x3FF) << 12) | ((typ & 0xF) << 8) | (color8 & 0xFF)

geom = []
for i in range(0, len(edges) - 1, 2):
    x0, y0 = edges[i]
    x1, y1 = edges[i+1]
    x0 = max(0, min(WIDTH-1, x0))
    x1 = max(0, min(WIDTH-1, x1))
    y0 = max(0, min(HEIGHT-1, y0))
    y1 = max(0, min(HEIGHT-1, y1))
    y0 = HEIGHT - 1 - y0
    y1 = HEIGHT - 1 - y1
    geom.append(pack_geom(x0, y0, 0, ROAD_COLOR8))
    geom.append(pack_geom(x1, y1, 0, ROAD_COLOR8))
for (x, y) in nodes:
    x = max(0, min(WIDTH-1, x))
    y = max(0, min(HEIGHT-1, y))
    yi = HEIGHT - 1 - y
    geom.append(pack_geom(x, yi, 1, NODE_COLOR8))

with open(GEOM_PATH, 'w', encoding='utf-8') as f:
    f.write('memory_initialization_radix=16;\n')
    f.write('memory_initialization_vector=\n')
    for i, v in enumerate(geom):
        token = format(v & 0xFFFFFFFF, '08X')
        end = ',' if i < len(geom)-1 else ';'
        f.write(token + end + '\n')
print('WROTE', GEOM_PATH, 'entries=', len(geom))

# Create a small vehicles template COE (editable at runtime)
VEH_PATH = os.path.join(OUT_DIR, 'vehicles_template.coe')
# Format: each vehicle entry: x:10 y:10 id:6 speed:6 => packed into 32 bits (simple template)
with open(VEH_PATH, 'w', encoding='utf-8') as f:
    f.write('memory_initialization_radix=16;\n')
    f.write('memory_initialization_vector=\n')
    # write zero entries for N vehicles (user can edit)
    N = 64
    for i in range(N):
        f.write('00000000,' + '\n')
    f.write(';\n')
print('WROTE', VEH_PATH, 'entries=', N)

print('\nDone. Outputs in:', OUT_DIR)
