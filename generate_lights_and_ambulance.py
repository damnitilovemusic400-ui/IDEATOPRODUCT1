#!/usr/bin/env python3
"""
Generate `lights.coe` (per-junction traffic light state+timer) and
`vehicle_ambulance.coe` (per-frame single ambulance entry) for FPGA runtime.

Lights format (16-bit word, hex):
 - bits[15:14] = state (0=RED,1=GREEN,2=YELLOW)
 - bits[13:0]  = timer (frames remaining)

Ambulance vehicle format (32-bit word, hex) per frame:
 - bits[31:22] = x (10 bits)
 - bits[21:12] = y (10 bits)  (screen coordinates, Y inverted already)
 - bits[11:6]  = id (6 bits)   (use 63 for ambulance)
 - bits[5:0]   = flags/speed (6 bits)  (LSB bit0 = blink flag toggles each frame)

Run this script in the same folder as `COE results` created earlier; outputs written to `COE results`.
"""
import os
import math
import random

# Config
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(SCRIPT_DIR, 'COE results')
if not os.path.exists(OUT_DIR):
    os.makedirs(OUT_DIR)

N_FRAMES = 2000
AMB_ID = 63
BLINK_PERIOD = 20  # frames on/off
# light phase lengths (frames at 60Hz ~ 60 frames=1s)
GREEN_LEN = 120   # 2s
YELLOW_LEN = 30   # 0.5s
RED_LEN = 150     # 2.5s

# Helpers

def read_nodes_coe(path):
    if not os.path.exists(path):
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

NODES_PATH = os.path.join(SCRIPT_DIR, 'map_data.coe')
if not os.path.exists(NODES_PATH):
    # try parent folder
    NODES_PATH = os.path.join(os.path.dirname(SCRIPT_DIR), 'map_data.coe')

nodes = read_nodes_coe(NODES_PATH)
if not nodes:
    print('No nodes found at', NODES_PATH)
    raise SystemExit(1)

num_nodes = len(nodes)
print('nodes:', num_nodes)

# Build lights table: assign random starting phase and timer
lights = []
for i in range(num_nodes):
    # choose starting phase offset to avoid global sync
    phase = random.choice([0,1,2])  # 0=RED,1=GREEN,2=YELLOW
    if phase == 0:
        timer = random.randint(30, RED_LEN)
    elif phase == 1:
        timer = random.randint(30, GREEN_LEN)
    else:
        timer = random.randint(10, YELLOW_LEN)
    lights.append((phase, timer))

# Write lights.coe (16-bit per entry)
lights_path = os.path.join(OUT_DIR, 'lights.coe')
with open(lights_path, 'w', encoding='utf-8') as f:
    f.write('memory_initialization_radix=16;\n')
    f.write('memory_initialization_vector=\n')
    for i, (state, timer) in enumerate(lights):
        word = ((state & 0x3) << 14) | (timer & 0x3FFF)
        f.write(format(word, '04X'))
        f.write(',' if i < num_nodes-1 else ';\n')
print('WROTE', lights_path, 'entries=', num_nodes)

# Ambulance simulation along straight-line path between two random nodes
start_idx = random.randrange(num_nodes)
end_idx = random.randrange(num_nodes)
while end_idx == start_idx:
    end_idx = random.randrange(num_nodes)
start = nodes[start_idx]
end = nodes[end_idx]
print('Ambulance path: from', start_idx, 'to', end_idx)

# Simulate moving straight line but stop at nodes if light is RED
# We'll define intermediate "nodes" only at start and end for this simple simulation
# speed: pixels per frame
speed = 2.0

def lerp(a, b, t):
    return a + (b - a) * t

x0, y0 = start
x1, y1 = end
# ensure coordinates inside 640x480
WIDTH = 640
HEIGHT = 480
x0 = max(0, min(WIDTH-1, x0))
y0 = max(0, min(HEIGHT-1, y0))
x1 = max(0, min(WIDTH-1, x1))
y1 = max(0, min(HEIGHT-1, y1))

# Simulation variables
posx = float(x0)
posy = float(y0)
dx = x1 - x0
dy = y1 - y0
dist = math.hypot(dx, dy)
if dist == 0:
    steps_total = 1
else:
    steps_total = int(dist / speed) + 1

frames = []
arrived = False
for frame in range(N_FRAMES):
    # blink flag toggles
    blink = (frame // (BLINK_PERIOD//2)) % 2
    if not arrived:
        # determine fraction along path
        if dist == 0:
            frac = 1.0
        else:
            frac = min(1.0, (frame * speed) / dist)
        # compute intended pos
        tx = lerp(x0, x1, frac)
        ty = lerp(y0, y1, frac)
        # if near end, consider arrival
        if frac >= 1.0 or math.hypot(tx - x1, ty - y1) < 1.0:
            posx = x1
            posy = y1
            arrived = True
        else:
            # check lights at next node (end only in this simple model)
            # We invert Y earlier when rendering; lights stored correspond to node indices
            # if approaching end and its light is RED, pause until green
            approaching_end = math.hypot(tx - x1, ty - y1) < 16.0
            if approaching_end:
                end_state, end_timer = lights[end_idx]
                if end_state == 0:  # RED
                    # pause (do not advance)
                    # we optionally decrement timers globally to simulate progression
                    pass
                else:
                    posx = tx
                    posy = ty
            else:
                posx = tx
                posy = ty
    # pack ambulance entry: x10,y10,id6,flags6
    xi = int(max(0, min(WIDTH-1, round(posx))))
    yi = int(max(0, min(HEIGHT-1, round(posy))))
    pack = ((xi & 0x3FF) << 22) | ((yi & 0x3FF) << 12) | ((AMB_ID & 0x3F) << 6) | ( (blink & 1) )
    frames.append(pack)
    # advance lights timers (simple global tick)
    new_lights = []
    for (state, timer) in lights:
        if timer <= 1:
            # advance state
            if state == 0:
                new_state = 1
                new_timer = GREEN_LEN
            elif state == 1:
                new_state = 2
                new_timer = YELLOW_LEN
            else:
                new_state = 0
                new_timer = RED_LEN
            new_lights.append((new_state, new_timer))
        else:
            new_lights.append((state, timer-1))
    lights = new_lights

# write vehicle_ambulance.coe (one entry per frame)
veh_path = os.path.join(OUT_DIR, 'vehicle_ambulance.coe')
with open(veh_path, 'w', encoding='utf-8') as f:
    f.write('memory_initialization_radix=16;\n')
    f.write('memory_initialization_vector=\n')
    for i, v in enumerate(frames):
        f.write(format(v & 0xFFFFFFFF, '08X'))
        f.write(',' if i < len(frames)-1 else ';\n')
print('WROTE', veh_path, 'frames=', len(frames))

# Update lights.coe with final lights state (so FPGA can start from a known state after simulation)
with open(lights_path, 'w', encoding='utf-8') as f:
    f.write('memory_initialization_radix=16;\n')
    f.write('memory_initialization_vector=\n')
    for i, (state, timer) in enumerate(lights):
        word = ((state & 0x3) << 14) | (timer & 0x3FFF)
        f.write(format(word, '04X'))
        f.write(',' if i < num_nodes-1 else ';\n')
print('UPDATED', lights_path)

print('\nDone. Outputs in:', OUT_DIR)
