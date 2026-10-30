#!/usr/bin/env python3
"""ASCII preview of a trinary PGM map (debug aid): # occupied, . free, ' ' unknown.

Usage: python3 tools/show_map.py src/fleetgraph_sim/maps/lab_floor.pgm [step]
"""
import sys

PIXEL_SYMBOLS = {0: '#', 254: '.', 205: ' '}

path = sys.argv[1]
step = int(sys.argv[2]) if len(sys.argv) > 2 else 4
with open(path, 'rb') as file:
    data = file.read()
header = [line for line in data.split(b'\n')[:4] if not line.startswith(b'#')]
width, height = map(int, header[1].split())
pixels = data[-width * height:]
for row in range(0, height, step):
    print(''.join(PIXEL_SYMBOLS.get(pixels[row * width + col], '?')
                  for col in range(0, width, step)))
