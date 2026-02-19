#!/usr/bin/env python3
"""Convert Xilinx COE files to simple hex .mem files for simulation/synthesis flows.

Usage: python coe_to_mem.py <source.coe> <out.mem>
If given a directory, converts all .coe files inside to .mem with same base name.
"""
import sys
import os

def coe_to_mem(src, dst):
    with open(src, 'r', encoding='utf-8') as f:
        text = f.read()
    for ch in [';', '\n', '\r']:
        text = text.replace(ch, ',')
    toks = [t.strip() for t in text.split(',') if t.strip()]
    vals = []
    for t in toks:
        if t.lower().startswith('memory_initialization'): continue
        tt = t
        if tt.lower().startswith('0x'):
            tt = tt[2:]
        # ensure fixed width by preserving length
        if all(c in '0123456789abcdefABCDEF' for c in tt):
            vals.append(tt.upper())
    with open(dst, 'w', encoding='utf-8') as f:
        for v in vals:
            f.write(v + '\n')

def main():
    if len(sys.argv) < 2:
        print('Usage: coe_to_mem.py <file_or_dir> [out_dir]')
        return
    src = sys.argv[1]
    outdir = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(src) if os.path.isfile(src) else src
    os.makedirs(outdir, exist_ok=True)
    if os.path.isdir(src):
        for f in os.listdir(src):
            if f.lower().endswith('.coe'):
                coe = os.path.join(src, f)
                out = os.path.join(outdir, os.path.splitext(f)[0] + '.mem')
                coe_to_mem(coe, out)
                print('WROTE', out)
    else:
        out = os.path.join(outdir, os.path.splitext(os.path.basename(src))[0] + '.mem')
        coe_to_mem(src, out)
        print('WROTE', out)

if __name__ == '__main__':
    main()
