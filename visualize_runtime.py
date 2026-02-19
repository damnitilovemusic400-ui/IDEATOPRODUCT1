#!/usr/bin/env python3
"""Create a PPM overlay of ambulance paths on top of framebuffer_heat.coe.
This avoids external imaging dependencies.
"""
import os

def read_framebuffer_coe(path, width=640, height=480):
    with open(path,'r',encoding='utf-8') as f:
        text = f.read()
    for ch in [';', '\n', '\r']:
        text = text.replace(ch, ',')
    toks = [t.strip() for t in text.split(',') if t.strip()]
    vals = []
    for t in toks:
        if t.lower().startswith('memory_initialization'): continue
        if all(c in '0123456789abcdefABCDEF' for c in t):
            vals.append(int(t,16) & 0xFFFF)
    # convert rgb565 to rgb888
    pixels = []
    for v in vals[:width*height]:
        r = ((v>>11) & 0x1F) * 255 // 31
        g = ((v>>5) & 0x3F) * 255 // 63
        b = (v & 0x1F) * 255 // 31
        pixels.append((r,g,b))
    # pad if necessary
    while len(pixels) < width*height:
        pixels.append((0,0,0))
    return pixels

def make_ppm_overlay(fb_coe, nodes, paths, out_ppm, width=640, height=480):
    pixels = read_framebuffer_coe(fb_coe, width, height)
    # draw paths: each path a color
    colors = [(255,0,0),(0,255,0),(0,0,255),(255,255,0),(255,0,255)]
    for pi,path in enumerate(paths):
        col = colors[pi % len(colors)]
        for ni in path:
            x,y = nodes[ni]
            if 0<=x<width and 0<=y<height:
                pixels[y*width + x] = col
                # mark small cross
                for dx in (-1,0,1):
                    for dy in (-1,0,1):
                        xx = x+dx; yy = y+dy
                        if 0<=xx<width and 0<=yy<height:
                            pixels[yy*width + xx] = col
    # write PPM
    with open(out_ppm, 'wb') as f:
        header = f'P6\n{width} {height}\n255\n'
        f.write(header.encode('ascii'))
        for (r,g,b) in pixels:
            f.write(bytes((r,g,b)))

__all__ = ['make_ppm_overlay']
