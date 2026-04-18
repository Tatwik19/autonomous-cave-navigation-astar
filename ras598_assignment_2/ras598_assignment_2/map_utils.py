#!/usr/bin/env python3

import ast
import math
import os
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class MapInfo:
    image_path: str
    resolution: float          # meters per pixel (from map.yaml)
    origin_x: float            # world-frame X of the grid's bottom-left corner
    origin_y: float            # world-frame Y of the grid's bottom-left corner
    image_width: int
    image_height: int
    grid_resolution: float     # meters per grid cell (0.2m per assignment spec)
    inflation_radius_m: float  # obstacle inflation radius in meters
    occupancy_grid: np.ndarray # 2D array, 0 = free, 1 = occupied
    grid_width: int
    grid_height: int


def load_map_yaml_manual(package_share: str):
    map_yaml_path = os.path.join(package_share, 'map.yaml')

    image_rel = None
    resolution = None
    origin = None

    with open(map_yaml_path, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('image:'):
                image_rel = line.split(':', 1)[1].strip()
            elif line.startswith('resolution:'):
                resolution = float(line.split(':', 1)[1].strip())
            elif line.startswith('origin:'):
                origin = ast.literal_eval(line.split(':', 1)[1].strip())

    if None in (image_rel, resolution, origin):
        raise RuntimeError(f'Failed to parse map.yaml at {map_yaml_path}')

    return os.path.join(package_share, image_rel), resolution, origin


def build_occupancy_grid(
    package_share: str,
    grid_resolution: float = 0.2,
    inflation_radius_m: float = 0.8,
) -> MapInfo:
    """
    Load the cave bitmap, inflate the walls, and rasterize into a coarse occupancy grid.

    Steps:
    1. Load the grayscale PNG. Dark pixels (< 127) are walls.
    2. Dilate the wall mask by inflation_radius_m to account for the robot footprint.
    3. Mark a coarse grid cell occupied if any inflated obstacle pixel lies inside it.

    Coordinate conventions:
    - Image: origin at top-left, Y increases downward.
    - World (Stage): origin at map center, Y increases upward.
    - Grid: indexed [gy][gx], origin at map bottom-left (= world origin from map.yaml).
    """
    image_path, map_res, origin = load_map_yaml_manual(package_share)

    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise RuntimeError(f'Could not load map image: {image_path}')

    h, w = img.shape

    wall_mask = img < 127

    inflate_px = math.ceil(inflation_radius_m / map_res)
    k = 2 * inflate_px + 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    inflated = cv2.dilate(wall_mask.astype(np.uint8), kernel, iterations=1).astype(bool)

    world_w = w * map_res
    world_h = h * map_res
    grid_w = int(math.ceil(world_w / grid_resolution))
    grid_h = int(math.ceil(world_h / grid_resolution))
    grid = np.zeros((grid_h, grid_w), dtype=np.uint8)

    for gy in range(grid_h):
        y0_w = gy * grid_resolution
        y1_w = min((gy + 1) * grid_resolution, world_h)

        py0 = max(0, h - int(math.ceil(y1_w / map_res)))
        py1 = min(h, h - int(math.floor(y0_w / map_res)))

        for gx in range(grid_w):
            x0_w = gx * grid_resolution
            x1_w = min((gx + 1) * grid_resolution, world_w)

            px0 = max(0, int(math.floor(x0_w / map_res)))
            px1 = min(w, int(math.ceil(x1_w / map_res)))

            if px0 < px1 and py0 < py1 and np.any(inflated[py0:py1, px0:px1]):
                grid[gy, gx] = 1

    return MapInfo(
        image_path=image_path,
        resolution=map_res,
        origin_x=origin[0],
        origin_y=origin[1],
        image_width=w,
        image_height=h,
        grid_resolution=grid_resolution,
        inflation_radius_m=inflation_radius_m,
        occupancy_grid=grid,
        grid_width=grid_w,
        grid_height=grid_h,
    )


def world_to_grid(wx: float, wy: float, map_info: MapInfo):
    """Convert world coordinates to occupancy grid indices."""
    gx = math.floor((wx - map_info.origin_x) / map_info.grid_resolution)
    gy = math.floor((wy - map_info.origin_y) / map_info.grid_resolution)
    return int(gx), int(gy)


def grid_to_world(gx: int, gy: int, map_info: MapInfo):
    """Convert grid indices to the world coordinates of the cell center."""
    wx = map_info.origin_x + (gx + 0.5) * map_info.grid_resolution
    wy = map_info.origin_y + (gy + 0.5) * map_info.grid_resolution
    return wx, wy


def is_in_bounds(gx: int, gy: int, map_info: MapInfo) -> bool:
    return 0 <= gx < map_info.grid_width and 0 <= gy < map_info.grid_height


def is_occupied(gx: int, gy: int, map_info: MapInfo) -> bool:
    if not is_in_bounds(gx, gy, map_info):
        return True
    return map_info.occupancy_grid[gy, gx] == 1