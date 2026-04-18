#!/usr/bin/env python3

import heapq
import math

from ras598_assignment_2.map_utils import is_in_bounds, is_occupied


def heuristic(a, b):
    """Euclidean distance between two grid cells. Admissible for an 8-connected grid."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def get_neighbors(node):
    """
    8-connected neighbors with their move costs.
    Cardinal moves cost 1.0, diagonal moves cost sqrt(2).
    """
    x, y = node
    return [
        (x + 1, y,     1.0),
        (x - 1, y,     1.0),
        (x,     y + 1, 1.0),
        (x,     y - 1, 1.0),
        (x + 1, y + 1, math.sqrt(2)),
        (x + 1, y - 1, math.sqrt(2)),
        (x - 1, y + 1, math.sqrt(2)),
        (x - 1, y - 1, math.sqrt(2)),
    ]


def reconstruct_path(came_from, end):
    path = [end]
    while end in came_from:
        end = came_from[end]
        path.append(end)
    path.reverse()
    return path


def astar_search(start, goal, map_info):
    """
    A* on the occupancy grid. Returns a list of grid cells from start to goal,
    or None if no path exists.
    """
    for label, pt in [('Start', start), ('Goal', goal)]:
        if not is_in_bounds(pt[0], pt[1], map_info):
            raise ValueError(f'{label} {pt} is out of grid bounds')
        if is_occupied(pt[0], pt[1], map_info):
            raise ValueError(f'{label} {pt} is inside an obstacle (after inflation)')

    open_heap = []
    heapq.heappush(open_heap, (heuristic(start, goal), start))

    came_from = {}
    g = {start: 0.0}
    closed = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)

        if current in closed:
            continue

        if current == goal:
            return reconstruct_path(came_from, current)

        closed.add(current)

        for nx, ny, step_cost in get_neighbors(current):
            nb = (nx, ny)

            if not is_in_bounds(nx, ny, map_info):
                continue
            if is_occupied(nx, ny, map_info):
                continue

            # Block diagonal moves that would clip a wall corner.
            if nx != current[0] and ny != current[1]:
                if is_occupied(nx, current[1], map_info) or is_occupied(current[0], ny, map_info):
                    continue

            g_new = g[current] + step_cost

            if nb not in g or g_new < g[nb]:
                came_from[nb] = current
                g[nb] = g_new
                heapq.heappush(open_heap, (g_new + heuristic(nb, goal), nb))

    return None


def bresenham_line(x0, y0, x1, y1):
    """
    Rasterize a line segment into grid cells using Bresenham's algorithm.
    """
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x1 > x0 else -1
    sy = 1 if y1 > y0 else -1

    if dy <= dx:
        err = dx / 2.0
        while x != x1:
            points.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            points.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy

    points.append((x1, y1))
    return points


def has_line_of_sight(a, b, map_info):
    """True if no occupied cells lie on the straight line from a to b."""
    for gx, gy in bresenham_line(a[0], a[1], b[0], b[1]):
        if is_occupied(gx, gy, map_info):
            return False
    return True


def prune_path(path, map_info):
    if path is None or len(path) <= 2:
        return path

    pruned = [path[0]]
    anchor = 0

    while anchor < len(path) - 1:
        best = anchor + 1

        for i in range(anchor + 1, len(path)):
            if has_line_of_sight(path[anchor], path[i], map_info):
                best = i

        pruned.append(path[best])
        anchor = best

    return pruned