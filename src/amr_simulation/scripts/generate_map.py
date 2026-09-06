#!/usr/bin/env python3
"""
Warehouse 2D Occupancy Grid Map Generator
Generates warehouse_map.pgm and warehouse_map.yaml based on warehouse SDF dimensions.
Enforces 12m x 12m boundaries and central 1.15m narrow bottleneck.
"""

import os

width = 240
height = 240
resolution = 0.05
origin_x = -6.0
origin_y = -6.0

def world_to_grid(x, y):
    gx = int((x - origin_x) / resolution)
    gy = int((y - origin_y) / resolution)
    return gx, gy

# 255 = free space, 0 = occupied
grid = [[255 for _ in range(width)] for _ in range(height)]

def draw_rect(x_min, x_max, y_min, y_max):
    gx_min, gy_min = world_to_grid(x_min, y_min)
    gx_max, gy_max = world_to_grid(x_max, y_max)
    for y in range(max(0, gy_min), min(height, gy_max)):
        for x in range(max(0, gx_min), min(width, gx_max)):
            grid[y][x] = 0

# Walls
draw_rect(-6.0, 6.0, 5.8, 6.0) # North
draw_rect(-6.0, 6.0, -6.0, -5.8) # South
draw_rect(5.8, 6.0, -6.0, 6.0) # East
draw_rect(-6.0, -5.8, -6.0, 6.0) # West

# Choke points (0.4m width, 1.15m gap roughly y=-0.66 to y=0.66)
draw_rect(-0.2, 0.2, 0.66, 6.0) # Top
draw_rect(-0.2, 0.2, -6.0, -0.66) # Bottom

script_dir = os.path.dirname(os.path.abspath(__file__))
pkg_dir = os.path.dirname(script_dir)
maps_dir = os.path.join(pkg_dir, 'maps')
os.makedirs(maps_dir, exist_ok=True)

pgm_path = os.path.join(maps_dir, 'warehouse_map.pgm')
yaml_path = os.path.join(maps_dir, 'warehouse_map.yaml')

with open(pgm_path, 'w') as f:
    f.write(f"P2\n{width} {height}\n255\n")
    for y in range(height-1, -1, -1):
        row = " ".join(str(grid[y][x]) for x in range(width))
        f.write(row + "\n")

with open(yaml_path, 'w') as f:
    f.write(f"image: warehouse_map.pgm\n")
    f.write(f"mode: trinary\n")
    f.write(f"resolution: {resolution}\n")
    f.write(f"origin: [{origin_x}, {origin_y}, 0]\n")
    f.write(f"negate: 0\n")
    f.write(f"occupied_thresh: 0.65\n")
    f.write(f"free_thresh: 0.25\n")

print(f"Generated {pgm_path} and {yaml_path}")
