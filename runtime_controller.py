#!/usr/bin/env python3
"""
Runtime controller (Stage-2).
- Reads `route_log.txt` and `vehicle_ambulance_path.coe` to replay ambulance movement.
- Updates lights state with preemption window while ambulance moves.
- Emits `lights_runtime.coe`, `ambulance_runtime.coe`, and `runtime_log.txt` into `COE results`.

Run from the `sumoxml` folder.
"""
import os
import math
from collections import defaultdict

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(SCRIPT_DIR, 'COE results')

NODES_COE = os.path.join(SCRIPT_DIR, 'map_data.coe')
ORIG_LIGHTS = os.path.join(OUT_DIR, 'lights.coe')
PATH_COE = os.path.join(OUT_DIR, 'vehicle_ambulance_path.coe')
ROUTE_LOG = os.path.join(OUT_DIR, 'route_log.txt')

def parse_coe_hex(path):
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
    vals = parse_coe_hex(path)
    nodes = []
    for v in vals:
        x = (v>>10)&0x3FF
        y = v & 0x3FF
        nodes.append((x,y))
    return nodes

def load_route_nodes_from_log(path):
    if not os.path.exists(path):
        return None
    with open(path,'r',encoding='utf-8') as f:
        txt = f.read()
    for line in txt.splitlines():
        if line.startswith('path_nodes='):
            rhs = line.split('=',1)[1].strip()
            # expect python list like [1,2,3]
            rhs = rhs.strip()
            if rhs.startswith('[') and rhs.endswith(']'):
                inner = rhs[1:-1].strip()
                if not inner:
                    return []
                parts = [p.strip() for p in inner.split(',')]
                return [int(p) for p in parts]
    return None

def main():
    nodes = load_nodes(NODES_COE)
    N = len(nodes)

    # load original lights
    full_lights = [[0,60] for _ in range(N)]
    if os.path.exists(ORIG_LIGHTS):
        vals = parse_coe_hex(ORIG_LIGHTS)
        for i,v in enumerate(vals[:N]):
            st = (v>>14)&0x3
            tm = v & 0x3FFF
            full_lights[i] = [st, tm]

    # load path (node indices) from route_log
    path_nodes = load_route_nodes_from_log(ROUTE_LOG)
    if path_nodes is None:
        print('route_log.txt missing or malformed — attempting to read path COE coords instead')
        # fallback: read path COE coords and match to nearest node by coord
        path_vals = parse_coe_hex(PATH_COE)
        # convert to coords
        path_nodes = []
        coord_to_idx = {nodes[i]: i for i in range(len(nodes))}
        for w in path_vals:
            x = (w>>22)&0x3FF
            y = (w>>12)&0x3FF
            idx = coord_to_idx.get((x,y), None)
            if idx is None:
                # find nearest
                best = None; bd=1e9
                for i,p in enumerate(nodes):
                    d = (p[0]-x)**2 + (p[1]-y)**2
                    if d<bd: bd=d; best=i
                idx = best
            path_nodes.append(idx)

    if not path_nodes:
        raise SystemExit('No path nodes found')

    # simulation: move ambulance along path, apply preemption window
    preempt_count = max(1, int(len(path_nodes)*0.15))
    ticks = len(path_nodes)

    # keep ambulance frames (packed x,y like earlier)
    ambulance_frames = []

    # simulate timers: decrement timers each tick (if >0), and apply preempt override
    for t in range(ticks):
        current = path_nodes[t]
        # decrement timers
        for i in range(N):
            if full_lights[i][1] > 0:
                full_lights[i][1] = max(0, full_lights[i][1]-1)
        # apply preempt to upcoming nodes
        for ni in path_nodes[t:t+preempt_count]:
            full_lights[ni] = [1, 60]
        # stash ambulance frame
        x,y = nodes[current]
        word = ((x & 0x3FF) << 22) | ((y & 0x3FF) << 12)
        ambulance_frames.append(word & 0xFFFFFFFF)

    # write ambulance_runtime.coe
    amb_out = os.path.join(OUT_DIR, 'ambulance_runtime.coe')
    with open(amb_out,'w',encoding='utf-8') as f:
        f.write('memory_initialization_radix=16;\n')
        f.write('memory_initialization_vector=\n')
        for i,w in enumerate(ambulance_frames):
            f.write(format(w,'08X'))
            f.write(',' if i < len(ambulance_frames)-1 else ';\n')
    print('WROTE', amb_out)

    # write lights_runtime.coe (final state)
    lights_out = os.path.join(OUT_DIR, 'lights_runtime.coe')
    with open(lights_out,'w',encoding='utf-8') as f:
        f.write('memory_initialization_radix=16;\n')
        f.write('memory_initialization_vector=\n')
        for i,(st,tm) in enumerate(full_lights):
            word = ((st & 0x3) << 14) | (tm & 0x3FFF)
            f.write(format(word,'04X'))
            f.write(',' if i < N-1 else ';\n')
    print('WROTE', lights_out)

    # runtime log
    rlog = os.path.join(OUT_DIR, 'runtime_log.txt')
    with open(rlog,'w',encoding='utf-8') as f:
        f.write(f'ticks={ticks}\n')
        f.write(f'preempt_count={preempt_count}\n')
        f.write(f'path_nodes={path_nodes}\n')
    print('WROTE', rlog)

if __name__ == '__main__':
    main()
