#!/usr/bin/env python3
"""Example PVC-pipe course for testing Lab 2 against the CrazySim SITL sim.

Companion to ``worlds/mujoco/scene_course.xml``: that file is what MuJoCo
renders and collides with; this file is the SAME course expressed as the
numbers Lab 2 Part III asks you to measure, so the notebook plans in exactly
the world the simulator flies in.

Frame (see also the scene's header comment)
-------------------------------------------
The drone spawns at world ``(x, y) = (0, 0)`` facing +X and its onboard
estimator is zeroed at the start, so the arena frame the lab uses -- origin at
the start, ``+x`` forward, ``+y`` left -- is *identical* to the MuJoCo world
frame here. With the drone's origin at ``(0, 0)`` there is no arena->drone
conversion to do, but the general rule is still ``drone_xy = arena_xy - start``.

Numbers to paste into Part III
------------------------------
    ft = 1.0                     # this course is defined in metres already

    width  = 3.0                 # forward travel  (arena +x)
    height = 3.0                 # side-to-side    (arena +y)

    obstacles = [
        (np.array([0.9, 1.0]), 0.20),
        (np.array([1.5, 0.5]), 0.20),
        (np.array([1.5, 1.5]), 0.20),
        (np.array([2.1, 1.0]), 0.20),
        (np.array([0.9, 2.4]), 0.20),
        (np.array([1.9, 2.6]), 0.20),
        (np.array([2.6, 1.8]), 0.20),
    ]   # radius 0.20 = 0.05 pipe + 0.15 inflation

    goal   = (np.array([2.55, 2.55]), 0.30)
    origin = np.array([0.0, 0.0])

The obstacle radii are *inflated*: the pipes are 0.05 m in radius, plus a
0.15 m buffer, so a path found here clears every pipe by >= ~0.15 m of real
surface -- the margin the lab asks you to add.

Note this file is a course *definition*, not a Lab 2 solution: it contains no
RRT. Run it to print the block above and to sanity-check that the course is
connected (via a grid BFS, an algorithm unrelated to RRT).
"""
from __future__ import annotations

from collections import deque

import numpy as np

# --- Course geometry (arena frame, metres) ---------------------------------
CAGE_WIDTH = 3.0
CAGE_HEIGHT = 3.0

PIPE_RADIUS = 0.05          # true PVC pipe radius (what the scene renders)
INFLATION = 0.15            # planner buffer the lab asks for
PLANNING_RADIUS = PIPE_RADIUS + INFLATION

ORIGIN = np.array([0.0, 0.0])
GOAL_CENTER = np.array([2.55, 2.55])
GOAL_RADIUS = 0.30

PIPE_CENTERS = [
    np.array([0.9, 1.0]),
    np.array([1.5, 0.5]),
    np.array([1.5, 1.5]),
    np.array([2.1, 1.0]),
    np.array([0.9, 2.4]),
    np.array([1.9, 2.6]),
    np.array([2.6, 1.8]),
]

# What the planner sees: every pipe inflated by INFLATION.
OBSTACLES = [(c, PLANNING_RADIUS) for c in PIPE_CENTERS]
GOAL = (GOAL_CENTER, GOAL_RADIUS)


def part_three_block() -> str:
    """Return the ready-to-paste Part III measurement block."""
    obs = ",\n        ".join(
        f"(np.array([{c[0]:.2f}, {c[1]:.2f}]), {PLANNING_RADIUS:.2f})"
        for c in PIPE_CENTERS
    )
    return (
        "width  = %.1f\n"
        "height = %.1f\n\n"
        "obstacles = [\n        %s,\n    ]\n\n"
        "goal   = (np.array([%.2f, %.2f]), %.2f)\n"
        "origin = np.array([%.2f, %.2f])"
        % (CAGE_WIDTH, CAGE_HEIGHT, obs,
           GOAL_CENTER[0], GOAL_CENTER[1], GOAL_RADIUS,
           ORIGIN[0], ORIGIN[1])
    )


def _grid_solvable(resolution: float = 0.05):
    """4-connected grid BFS on the inflated free space (NOT RRT).

    Returns (reachable, cells, path) where path is a list of (x, y) waypoints
    from origin to the goal region, or None.
    """
    nx = int(round(CAGE_WIDTH / resolution)) + 1
    ny = int(round(CAGE_HEIGHT / resolution)) + 1
    xs = np.linspace(0.0, CAGE_WIDTH, nx)
    ys = np.linspace(0.0, CAGE_HEIGHT, ny)
    free = np.ones((nx, ny), dtype=bool)
    for c in PIPE_CENTERS:
        d2 = (xs[:, None] - c[0]) ** 2 + (ys[None, :] - c[1]) ** 2
        free &= d2 >= PLANNING_RADIUS ** 2

    si, sj = 0, 0  # origin (0, 0)
    assert free[si, sj], "origin is not free"
    prev = {(si, sj): None}
    q = deque([(si, sj)])
    goal_cell = None
    while q:
        i, j = q.popleft()
        if np.hypot(xs[i] - GOAL_CENTER[0], ys[j] - GOAL_CENTER[1]) < GOAL_RADIUS:
            goal_cell = (i, j)
            break
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ni, nj = i + di, j + dj
            if 0 <= ni < nx and 0 <= nj < ny and free[ni, nj] and (ni, nj) not in prev:
                prev[(ni, nj)] = (i, j)
                q.append((ni, nj))
    if goal_cell is None:
        return False, free.sum(), None
    path = []
    node = goal_cell
    while node is not None:
        path.append((xs[node[0]], ys[node[1]]))
        node = prev[node]
    path.reverse()
    return True, free.sum(), path


def _ascii_map(rows: int = 26, cols: int = 52) -> str:
    grid = [[" "] * cols for _ in range(rows)]
    for c in PIPE_CENTERS:
        cx = int(round(c[0] / CAGE_WIDTH * (cols - 1)))
        cy = int(round(c[1] / CAGE_HEIGHT * (rows - 1)))
        grid[rows - 1 - cy][cx] = "O"
    grid[rows - 1 - 0][0] = "S"
    gx = int(round(GOAL_CENTER[0] / CAGE_WIDTH * (cols - 1)))
    gy = int(round(GOAL_CENTER[1] / CAGE_HEIGHT * (rows - 1)))
    grid[rows - 1 - gy][gx] = "G"
    return "\n".join("".join(r) for r in grid)


def main() -> None:
    print("Arena frame: origin (0,0)=start, +x forward, +y left")
    print(f"cage {CAGE_WIDTH} x {CAGE_HEIGHT} m | "
          f"pipe radius {PIPE_RADIUS} m | inflated to {PLANNING_RADIUS} m\n")
    print(_ascii_map())
    ok, cells, path = _grid_solvable()
    print(f"\nfree grid cells (5 cm): {cells}")
    if ok:
        print(f"grid BFS: CONNECTED  (waypoints={len(path)}, "
              f"length~{sum(np.hypot(*(np.subtract(b, a))) for a, b in zip(path, path[1:])):.2f} m)")
    else:
        print("grid BFS: NOT connected -- a point robot cannot reach the goal")
    print("\n--- Part III block ---")
    print(part_three_block())
    print("\n--- run the matching scene in the simulator ---")
    print("  pixi run -e sim sim-course      # from the repo root")
    print("  # or: pixi run -e sim sim -- -s \"$PWD/worlds/mujoco/scene_course.xml\"")


if __name__ == "__main__":
    main()
