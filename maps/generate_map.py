"""
maps/generate_map.py
────────────────────
Generates simple_map.pgm — a 200×200 greyscale occupancy image.

  White  (255) = free space
  Black  (0)   = obstacle / wall
  Grey   (128-254) = unknown (not used here)

Arena layout (10×10 m, 0.05 m/px):
  - 1-pixel border walls on all sides
  - 5 rectangular obstacles matching Isaac Sim OBSTACLE_SPECS

Run:
    python3 maps/generate_map.py
"""

import struct
import os

WIDTH  = 200   # pixels  (200 × 0.05 m = 10 m)
HEIGHT = 200

# Origin offset: pixel (col, row) for world (x=0, y=0)
# origin = [-5, -5]  →  world x=0 is at col 100, world y=0 is at row 100
ORIGIN_X = -5.0
ORIGIN_Y = -5.0
RES = 0.05  # m/px

def world_to_pixel(wx, wy):
    col = int((wx - ORIGIN_X) / RES)
    row = int(HEIGHT - 1 - (wy - ORIGIN_Y) / RES)  # y-axis flipped in image
    return col, row

# Isaac Sim obstacle specs: (cx, cy, half_w, half_d, half_h)
OBSTACLES = [
    ( 2.0,  1.0, 0.20, 0.20),
    (-2.0,  1.5, 0.15, 0.40),
    ( 0.5, -2.5, 0.30, 0.15),
    (-1.5, -1.0, 0.25, 0.25),
    ( 3.0, -1.5, 0.15, 0.50),
]

# Build pixel grid (row-major, 0 = black = occupied)
grid = [[255] * WIDTH for _ in range(HEIGHT)]

# Border walls (2 px thick)
for r in range(HEIGHT):
    for c in range(WIDTH):
        if r < 2 or r >= HEIGHT - 2 or c < 2 or c >= WIDTH - 2:
            grid[r][c] = 0

# Obstacle rectangles
for cx, cy, hw, hd in OBSTACLES:
    c0, r0 = world_to_pixel(cx - hw, cy + hd)
    c1, r1 = world_to_pixel(cx + hw, cy - hd)
    # clamp to grid
    c_lo, c_hi = max(0, min(c0, c1)), min(WIDTH-1,  max(c0, c1))
    r_lo, r_hi = max(0, min(r0, r1)), min(HEIGHT-1, max(r0, r1))
    for r in range(r_lo, r_hi + 1):
        for c in range(c_lo, c_hi + 1):
            grid[r][c] = 0

# Write PGM (binary P5 format)
out_path = os.path.join(os.path.dirname(__file__), "simple_map.pgm")
with open(out_path, "wb") as f:
    header = f"P5\n{WIDTH} {HEIGHT}\n255\n"
    f.write(header.encode("ascii"))
    for row in grid:
        f.write(bytes(row))

print(f"Wrote {out_path}  ({WIDTH}×{HEIGHT} px, {WIDTH*RES:.0f}×{HEIGHT*RES:.0f} m)")
