# SPDX-License-Identifier: MulanPSL-2.0
"""Frontier extraction and footprint-safe approach selection on live occupancy grids."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import time
from typing import List, Optional, Tuple

import numpy as np


OCC_THRESH = 50  # ≥ this counts as obstacle (matches nav2 convention)


@dataclass
class GridView:
    """Numpy-friendly view of nav_msgs/OccupancyGrid."""
    data: np.ndarray         # shape (h, w), int8 in [-1, 100]
    resolution: float        # m / cell
    origin_x: float
    origin_y: float
    width: int
    height: int

    @classmethod
    def from_msg(cls, msg) -> "GridView":
        """Read signed ROS occupancy values without converting negative values to bytes."""
        h, w = int(msg.info.height), int(msg.info.width)
        arr = np.asarray(msg.data, dtype=np.int8).reshape(h, w)
        return cls(
            data=arr,
            resolution=float(msg.info.resolution),
            origin_x=float(msg.info.origin.position.x),
            origin_y=float(msg.info.origin.position.y),
            width=w, height=h,
        )

    def cell_to_world(self, cx: int, cy: int) -> Tuple[float, float]:
        return (self.origin_x + (cx + 0.5) * self.resolution,
                self.origin_y + (cy + 0.5) * self.resolution)

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        return (math.floor((x - self.origin_x) / self.resolution),
                math.floor((y - self.origin_y) / self.resolution))

    def in_bounds(self, cx: int, cy: int) -> bool:
        return 0 <= cx < self.width and 0 <= cy < self.height


@dataclass
class FrontierCluster:
    centroid_xy: Tuple[float, float]   # world coords
    size: int                          # cell count
    cell_indices: np.ndarray           # (N, 2) int — for debugging / viz


def find_frontier_cells(gv: GridView) -> np.ndarray:
    """Return (N, 2) array of (cx, cy) for cells that are free AND
    have at least one unknown 4-neighbour. Vectorised via shifted
    masks to avoid per-cell python loops."""
    g = gv.data
    free    = (g == 0)
    unknown = (g == -1)

    # Pad unknown by 1 in each direction; OR them and intersect with free.
    h, w = g.shape
    has_unknown_neighbour = np.zeros_like(free, dtype=bool)
    has_unknown_neighbour[1:, :]   |= unknown[:-1, :]   # neighbour above
    has_unknown_neighbour[:-1, :]  |= unknown[1:, :]    # below
    has_unknown_neighbour[:, 1:]   |= unknown[:, :-1]   # left
    has_unknown_neighbour[:, :-1]  |= unknown[:, 1:]    # right

    frontier_mask = free & has_unknown_neighbour
    yy, xx = np.where(frontier_mask)
    return np.stack([xx, yy], axis=1)  # (N, 2) as (cx, cy)


def cluster_frontiers(cells: np.ndarray, min_size: int = 3,
                       max_link_cells: int = 2) -> List[FrontierCluster]:
    """Connected-components style clustering with 8-neighbour adjacency
    extended by `max_link_cells` (cells within this Chebyshev distance
    are merged into the same cluster). This is cheaper than a real
    DBSCAN since we already have integer grid coords.

    Drops clusters smaller than `min_size` cells — those are usually
    noise from boundary cells against partially-mapped obstacles.
    """
    if cells.size == 0:
        return []

    # Bucket into a sparse grid for fast neighbour lookup.
    cell_set = {(int(c[0]), int(c[1])): i for i, c in enumerate(cells)}
    parent = list(range(len(cells)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    r = max_link_cells
    for (cx, cy), idx in cell_set.items():
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx == 0 and dy == 0:
                    continue
                nb = (cx + dx, cy + dy)
                j = cell_set.get(nb)
                if j is not None:
                    union(idx, j)

    groups: dict[int, list[int]] = {}
    for i in range(len(cells)):
        groups.setdefault(find(i), []).append(i)

    clusters: List[FrontierCluster] = []
    for _, members in groups.items():
        if len(members) < min_size:
            continue
        member_arr = cells[members]
        # Centroid in world frame computed by caller (needs GridView);
        # here we just produce cell-space mean and let the caller
        # convert.
        clusters.append(FrontierCluster(
            centroid_xy=(float(member_arr[:, 0].mean()),
                         float(member_arr[:, 1].mean())),  # cell-space, will convert
            size=len(members),
            cell_indices=member_arr,
        ))
    return clusters


def footprint_free(gv: GridView, radius_m: float) -> np.ndarray:
    """Erode known-free cells by a disk, including each blocked cell's full area."""
    if not math.isfinite(radius_m) or radius_m <= 0:
        raise ValueError("robot radius must be finite and positive")
    free = gv.data == 0
    cells = math.ceil(radius_m / gv.resolution + 0.5)
    padded = np.pad(free, cells, constant_values=False)
    safe = free.copy()
    for dy in range(-cells, cells + 1):
        for dx in range(-cells, cells + 1):
            # Nearest point of a neighbouring cell square to this cell centre.
            distance = math.hypot(max(abs(dx) - 0.5, 0), max(abs(dy) - 0.5, 0))
            if distance * gv.resolution <= radius_m:
                safe &= padded[cells + dy:cells + dy + gv.height,
                               cells + dx:cells + dx + gv.width]
    return safe


def grid_distances(mask: np.ndarray, seeds, max_steps=None) -> np.ndarray:
    """Four-connected breadth-first distances; blocked cells never enter the queue."""
    distances = np.full(mask.shape, -1, dtype=np.int32)
    pending = deque()
    height, width = mask.shape
    for x, y in seeds:
        x, y = int(x), int(y)
        if 0 <= x < width and 0 <= y < height and mask[y, x] and distances[y, x] < 0:
            distances[y, x] = 0
            pending.append((x, y))
    while pending:
        x, y = pending.popleft()
        steps = int(distances[y, x]) + 1
        if max_steps is not None and steps > max_steps:
            continue
        for xx, yy in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
            if (0 <= xx < width and 0 <= yy < height and mask[yy, xx]
                    and distances[yy, xx] < 0):
                distances[yy, xx] = steps
                pending.append((xx, yy))
    return distances


def is_target_safe(gv: GridView, wx: float, wy: float,
                   safe_radius_m: float = math.hypot(0.35, 0.20)) -> bool:
    """Require the complete conservative footprint to lie in known-free space."""
    cx, cy = gv.world_to_cell(wx, wy)
    return gv.in_bounds(cx, cy) and bool(footprint_free(gv, safe_radius_m)[cy, cx])


def score_clusters(clusters: List[FrontierCluster], gv: GridView,
                   robot_xy: Tuple[float, float], *,
                   max_distance_m: float = 8.0,
                   visited_cells: Optional[set] = None,
                   robot_radius_m: float = math.hypot(0.35, 0.20),
                   approach_distance_m: float = 0.8,
                   failed_goals=(), failed_goal_radius_m: float = 0.6,
                   now: Optional[float] = None,
                   min_goal_distance_m: float = 0.35
                   ) -> List[Tuple[float, FrontierCluster]]:
    """Rank known-free approach cells reachable with footprint clearance.

    Cluster means are never navigation goals. Candidates must connect to an
    actual frontier member through known-free cells within approach_distance_m.
    Failed world-coordinate neighbourhoods expire without depending on grid origin.
    """
    safe = footprint_free(gv, robot_radius_m)
    seed = gv.world_to_cell(*robot_xy)
    travel = grid_distances(safe, [seed], max_steps=math.floor(max_distance_m / gv.resolution))
    raw_connected = grid_distances(gv.data == 0, [seed]) >= 0
    yy, xx = np.indices(safe.shape)
    wx = gv.origin_x + (xx + 0.5) * gv.resolution
    wy = gv.origin_y + (yy + 0.5) * gv.resolution
    eligible = (travel >= 0) & (np.hypot(wx - robot_xy[0], wy - robot_xy[1]) >= min_goal_distance_m)
    for cx, cy in visited_cells or ():
        if gv.in_bounds(cx, cy):
            eligible[cy, cx] = False
    at = time.monotonic() if now is None else now
    for x, y, expires in failed_goals:
        if expires > at:
            eligible &= np.hypot(wx - x, wy - y) > failed_goal_radius_m
    scored = []
    for cluster in clusters:
        members = cluster.cell_indices
        members = members[raw_connected[members[:, 1], members[:, 0]]]
        if len(members) == 0:
            continue
        approach = grid_distances(gv.data == 0, members,
                                  max_steps=math.floor(approach_distance_m / gv.resolution))
        candidates = eligible & (approach >= 0)
        if not np.any(candidates):
            continue
        # Approach the observed boundary, then prefer the shortest feasible travel.
        nearest = int(approach[candidates].min())
        candidates &= approach <= nearest + 1
        costs = np.where(candidates, travel + approach, np.iinfo(np.int32).max)
        cy, cx = np.unravel_index(int(costs.argmin()), costs.shape)
        score = len(members) / (1.0 + float(costs[cy, cx]) * gv.resolution)
        scored.append((score, FrontierCluster(
            centroid_xy=gv.cell_to_world(int(cx), int(cy)),
            size=len(members), cell_indices=members)))
    scored.sort(key=lambda item: item[0], reverse=True)
    return scored


def pick_target(gv: GridView, robot_xy: Tuple[float, float], *,
                min_size: int = 3, max_distance_m: float = 8.0,
                visited_cells: Optional[set] = None, **selection
                ) -> Optional[FrontierCluster]:
    """Select a reachable free approach; no candidate does not imply completion."""
    cells = find_frontier_cells(gv)
    if cells.size == 0:
        return None
    scored = score_clusters(cluster_frontiers(cells, min_size=min_size), gv,
                            robot_xy, max_distance_m=max_distance_m,
                            visited_cells=visited_cells, **selection)
    return scored[0][1] if scored else None


def total_frontier_count(gv: GridView, min_size: int = 3) -> int:
    """For status reporting: how many frontier clusters remain that
    are large enough to warrant another exploration round."""
    cells = find_frontier_cells(gv)
    if cells.size == 0:
        return 0
    return len(cluster_frontiers(cells, min_size=min_size))


def mapped_free_area_m2(gv: GridView) -> float:
    """How much area (m²) is currently mapped as free."""
    free_cells = int(np.sum(gv.data == 0))
    return free_cells * (gv.resolution ** 2)
