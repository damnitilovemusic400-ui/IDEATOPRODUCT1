#!/usr/bin/env python3
"""
Rasterize SUMO-based COE geometry into a 640x480 RGB565 framebuffer COE.
Reads `map_data_edges.coe` (pairs of 20-bit packed X/Y) for straight road segments
and `map_data.coe` for junction points. Outputs `framebuffer.coe` in the same folder.
"""
import os
import sys

# Config
WIDTH = 640
HEIGHT = 480
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
NODES_COE = os.path.join(SCRIPT_DIR, 'map_data.coe')
EDGES_COE = os.path.join(SCRIPT_DIR, 'map_data_edges.coe')
OUT_COE = os.path.join(SCRIPT_DIR, 'framebuffer.coe')

# Colors in RGB888
BG_RGB = (0, 0, 0)
ROAD_RGB = (200, 200, 200)
JUNCTION_RGB = (255, 0, 0)

# 8-bit color token for geometry COE (user-editable)
ROAD_COLOR8 = 0xC8
JUNCTION_COLOR8 = 0xF0

# Helpers
def rgb888_to_rgb565(r, g, b):
    r5 = (r * 31) // 255
    g6 = (g * 63) // 255
    b5 = (b * 31) // 255
    return (r5 << 11) | (g6 << 5) | b5

def read_coe_points(path):
    """Read COE containing packed 20-bit hex words and return list of (x,y) ints."""
    if not os.path.exists(path):
        return []
    with open(path, 'r', encoding='utf-8') as f:
        text = f.read()
    # Extract hex tokens (split by commas/newlines/semicolons)
    for ch in [';', '\n', '\r']:
        text = text.replace(ch, ',')
    tokens = [t.strip() for t in text.split(',') if t.strip()]
    # Skip header tokens that are not hex (like memory_initialization_radix=16 or memory_initialization_vector=)
    vals = []
    for t in tokens:
        # allow optional 0x prefix
        tt = t
        if tt.lower().startswith('memory_initialization_radix') or tt.lower().startswith('memory_initialization_vector'):
            continue
        if tt.lower().startswith('0x'):
            tt = tt[2:]
        # filter out anything that's not hex (defensive)
        if all(c in '0123456789abcdefABCDEF' for c in tt):
            try:
                v = int(tt, 16)
            except Exception:
                continue
            x = (v >> 10) & 0x3FF
            y = v & 0x3FF
            vals.append((x, y))
    return vals

# Simple raster primitives
def set_pixel(buf, x, y, color):
    if 0 <= x < WIDTH and 0 <= y < HEIGHT:
        buf[y * WIDTH + x] = color

def draw_line(buf, x0, y0, x1, y1, color):
    # Bresenham's algorithm
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


def main():
    nodes = read_coe_points(NODES_COE)
    edges = read_coe_points(EDGES_COE)
    if not edges and not nodes:
        print('No input COE found:', NODES_COE, EDGES_COE)
        return
    print(f'Read {len(nodes)} nodes and {len(edges)} edge-points')

    # Create framebuffer (16-bit values)
    bg_color = rgb888_to_rgb565(*BG_RGB)
    road_color = rgb888_to_rgb565(*ROAD_RGB)
    junction_color = rgb888_to_rgb565(*JUNCTION_RGB)
    buf = [bg_color] * (WIDTH * HEIGHT)

    # Draw edges as consecutive endpoint pairs (0..1, 2..3, ...)
    for i in range(0, len(edges) - 1, 2):
        x0, y0 = edges[i]
        x1, y1 = edges[i+1]
        # clamp to framebuffer dimensions
        x0 = max(0, min(WIDTH-1, x0))
        x1 = max(0, min(WIDTH-1, x1))
        y0 = max(0, min(HEIGHT-1, y0))
        y1 = max(0, min(HEIGHT-1, y1))
        # Invert Y so SUMO/world Y-up becomes screen Y-down
        y0 = HEIGHT - 1 - y0
        y1 = HEIGHT - 1 - y1
        draw_line(buf, x0, y0, x1, y1, road_color)

    # Draw junctions as small circles (invert Y)
    for (x, y) in nodes:
        if 0 <= x < WIDTH and 0 <= y < HEIGHT:
            yi = HEIGHT - 1 - y
            draw_filled_circle(buf, x, yi, 2, junction_color)

    # Write COE: row-major top-to-bottom
    with open(OUT_COE, 'w', encoding='utf-8') as f:
        f.write('memory_initialization_radix=16;\n')
        f.write('memory_initialization_vector=\n')
        # Each pixel as 4-digit uppercase hex (zero-padded)
        total = WIDTH * HEIGHT
        for i, val in enumerate(buf):
            token = format(val & 0xFFFF, '04X')
            end = ',' if i < total-1 else ';'
            f.write(token + end + '\n')

    print('WROTE', OUT_COE, 'pixels=', WIDTH*HEIGHT)

    # Also emit a 32-bit geometry COE for on-FPGA rasterizers or simulators.
    GEOM_COE = os.path.join(SCRIPT_DIR, 'geometry32.coe')

    def pack_geom(x, y, typ, color8):
        # x:10, y:10, typ:4, color:8 => 32 bits
        return ((x & 0x3FF) << 22) | ((y & 0x3FF) << 12) | ((typ & 0xF) << 8) | (color8 & 0xFF)

    geom_entries = []
    # Edge endpoints as type=0
    for i in range(0, len(edges) - 1, 2):
        x0, y0 = edges[i]
        x1, y1 = edges[i+1]
        x0 = max(0, min(WIDTH-1, x0))
        x1 = max(0, min(WIDTH-1, x1))
        y0 = max(0, min(HEIGHT-1, y0))
        y1 = max(0, min(HEIGHT-1, y1))
        # invert Y for geometry as well
        y0 = HEIGHT - 1 - y0
        y1 = HEIGHT - 1 - y1
        geom_entries.append(pack_geom(x0, y0, 0, ROAD_COLOR8))
        geom_entries.append(pack_geom(x1, y1, 0, ROAD_COLOR8))

    # Nodes as type=1
    for (x, y) in nodes:
        x = max(0, min(WIDTH-1, x))
        y = max(0, min(HEIGHT-1, y))
        yi = HEIGHT - 1 - y
        geom_entries.append(pack_geom(x, yi, 1, JUNCTION_COLOR8))

    with open(GEOM_COE, 'w', encoding='utf-8') as f:
        f.write('memory_initialization_radix=16;\n')
        f.write('memory_initialization_vector=\n')
        for i, val in enumerate(geom_entries):
            token = format(val & 0xFFFFFFFF, '08X')
            end = ',' if i < len(geom_entries)-1 else ';'
            f.write(token + end + '\n')

    print('WROTE', GEOM_COE, 'entries=', len(geom_entries))

if __name__ == '__main__':
    main()
