#!/usr/bin/env python3
"""ASCII preview of a trinary PGM map (debug aid): # occupied, . free, ' ' unknown.

Usage: python3 tools/show_map.py src/fleetgraph_sim/maps/lab_floor.pgm [step]
"""
import sys

path = sys.argv[1]
step = int(sys.argv[2]) if len(sys.argv) > 2 else 4
data = open(path, 'rb').read()
header = [line for line in data.split(b'\n')[:4] if not line.startswith(b'#')]
w, h = map(int, header[1].split())
px = data[-w * h:]
for r in range(0, h, step):
    print(''.join({0: '#', 254: '.', 205: ' '}.get(px[r * w + c], '?')
                  for c in range(0, w, step)))
