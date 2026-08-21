# Copyright 2026 zzr
# SPDX-License-Identifier: Apache-2.0

"""Pure-Python frontier extraction and ranking for occupancy grids."""

from collections import deque
from dataclasses import dataclass
import heapq
import math
from pathlib import Path


Cell = tuple[int, int]


@dataclass(frozen=True)
class GridMap:
    """ROS OccupancyGrid data without a runtime dependency on ROS."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: tuple[int, ...]

    def __post_init__(self):
        expected = self.width * self.height
        if self.width <= 0 or self.height <= 0:
            raise ValueError('grid dimensions must be positive')
        if self.resolution <= 0.0:
            raise ValueError('grid resolution must be positive')
        if len(self.data) != expected:
            raise ValueError(
                f'grid data has {len(self.data)} cells, expected {expected}'
            )

    def contains(self, cell: Cell) -> bool:
        """Return whether a row/column cell is inside the map."""
        row, column = cell
        return 0 <= row < self.height and 0 <= column < self.width

    def value(self, cell: Cell) -> int:
        """Return the occupancy value for a valid cell."""
        row, column = cell
        return int(self.data[row * self.width + column])

    def cell_to_world(self, cell: Cell) -> tuple[float, float]:
        """Transform a cell centre through the OccupancyGrid origin pose."""
        row, column = cell
        local_x = (column + 0.5) * self.resolution
        local_y = (row + 0.5) * self.resolution
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        return (
            self.origin_x + cosine * local_x - sine * local_y,
            self.origin_y + sine * local_x + cosine * local_y,
        )

    def world_to_cell(self, x: float, y: float) -> Cell | None:
        """Transform a world coordinate into a grid cell."""
        dx = x - self.origin_x
        dy = y - self.origin_y
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        cell = (
            math.floor(local_y / self.resolution),
            math.floor(local_x / self.resolution),
        )
        return cell if self.contains(cell) else None


@dataclass(frozen=True)
class FrontierCandidate:
    """One connected frontier cluster and its selected navigation goal."""

    cells: tuple[Cell, ...]
    goal_cell: Cell
    x: float
    y: float
    yaw: float
    path_distance: float
    information_gain: float
    score: float


@dataclass(frozen=True)
class FrontierResult:
    """Frontier extraction output including diagnostic counts."""

    candidates: tuple[FrontierCandidate, ...]
    frontier_cells: tuple[Cell, ...]
    cluster_count: int
    rejected_small_clusters: int
    rejected_unreachable_clusters: int
    rejected_near_clusters: int
    rejected_clearance_clusters: int


FOUR_NEIGHBOURS = ((-1, 0), (1, 0), (0, -1), (0, 1))
EIGHT_NEIGHBOURS = (
    (-1, -1), (-1, 0), (-1, 1),
    (0, -1), (0, 1),
    (1, -1), (1, 0), (1, 1),
)


def _neighbours(grid: GridMap, cell: Cell, offsets):
    row, column = cell
    for row_offset, column_offset in offsets:
        neighbour = (row + row_offset, column + column_offset)
        if grid.contains(neighbour):
            yield neighbour


def _is_free(value: int, free_threshold: int) -> bool:
    return 0 <= value <= free_threshold


def _frontier_cells(grid: GridMap, free_threshold: int) -> set[Cell]:
    frontiers = set()
    for row in range(grid.height):
        for column in range(grid.width):
            cell = (row, column)
            if not _is_free(grid.value(cell), free_threshold):
                continue
            if any(
                grid.value(neighbour) < 0
                for neighbour in _neighbours(grid, cell, FOUR_NEIGHBOURS)
            ):
                frontiers.add(cell)
    return frontiers


def _cluster_frontiers(
    grid: GridMap,
    frontiers: set[Cell],
) -> list[tuple[Cell, ...]]:
    unvisited = set(frontiers)
    clusters = []
    while unvisited:
        seed = unvisited.pop()
        queue = deque([seed])
        cluster = [seed]
        while queue:
            current = queue.popleft()
            for neighbour in _neighbours(grid, current, EIGHT_NEIGHBOURS):
                if neighbour not in unvisited:
                    continue
                unvisited.remove(neighbour)
                queue.append(neighbour)
                cluster.append(neighbour)
        clusters.append(tuple(sorted(cluster)))
    return clusters


def _nearest_free_cell(
    grid: GridMap,
    start: Cell | None,
    free_threshold: int,
    search_radius_cells: int = 12,
) -> Cell | None:
    if start is None:
        return None
    if _is_free(grid.value(start), free_threshold):
        return start

    queue = deque([(start, 0)])
    visited = {start}
    while queue:
        current, distance = queue.popleft()
        if distance >= search_radius_cells:
            continue
        for neighbour in _neighbours(grid, current, EIGHT_NEIGHBOURS):
            if neighbour in visited:
                continue
            visited.add(neighbour)
            if _is_free(grid.value(neighbour), free_threshold):
                return neighbour
            queue.append((neighbour, distance + 1))
    return None


def _reachable_distances(
    grid: GridMap,
    start: Cell | None,
    free_threshold: int,
) -> dict[Cell, float]:
    start = _nearest_free_cell(grid, start, free_threshold)
    if start is None:
        return {}

    distances = {start: 0.0}
    queue = [(0.0, start)]
    while queue:
        current_distance, current = heapq.heappop(queue)
        if current_distance > distances[current]:
            continue
        current_row, current_column = current
        for neighbour in _neighbours(grid, current, EIGHT_NEIGHBOURS):
            if not _is_free(grid.value(neighbour), free_threshold):
                continue
            row, column = neighbour
            diagonal = row != current_row and column != current_column
            if diagonal:
                side_a = (row, current_column)
                side_b = (current_row, column)
                if not (
                    _is_free(grid.value(side_a), free_threshold)
                    and _is_free(grid.value(side_b), free_threshold)
                ):
                    continue
            step = math.sqrt(2.0) if diagonal else 1.0
            candidate_distance = current_distance + step
            if candidate_distance >= distances.get(neighbour, math.inf):
                continue
            distances[neighbour] = candidate_distance
            heapq.heappush(queue, (candidate_distance, neighbour))
    return distances


def _has_occupied_clearance(
    grid: GridMap,
    cell: Cell,
    occupied_threshold: int,
    clearance_cells: int,
) -> bool:
    if clearance_cells <= 0:
        return True
    row, column = cell
    clearance_squared = clearance_cells * clearance_cells
    for other_row in range(
        max(0, row - clearance_cells),
        min(grid.height, row + clearance_cells + 1),
    ):
        for other_column in range(
            max(0, column - clearance_cells),
            min(grid.width, column + clearance_cells + 1),
        ):
            delta_row = other_row - row
            delta_column = other_column - column
            if delta_row * delta_row + delta_column * delta_column > (
                clearance_squared
            ):
                continue
            if grid.value((other_row, other_column)) >= occupied_threshold:
                return False
    return True


def _goal_yaw(grid: GridMap, goal: Cell, cluster: tuple[Cell, ...]) -> float:
    del cluster
    unknown_world = []
    for neighbour in _neighbours(grid, goal, FOUR_NEIGHBOURS):
        if grid.value(neighbour) < 0:
            unknown_world.append(grid.cell_to_world(neighbour))
    if not unknown_world:
        return 0.0
    goal_x, goal_y = grid.cell_to_world(goal)
    unknown_x = sum(point[0] for point in unknown_world) / len(unknown_world)
    unknown_y = sum(point[1] for point in unknown_world) / len(unknown_world)
    return math.atan2(unknown_y - goal_y, unknown_x - goal_x)


def find_frontiers(
    grid: GridMap,
    robot_x: float,
    robot_y: float,
    *,
    free_threshold: int = 20,
    occupied_threshold: int = 65,
    min_cluster_size: int = 8,
    min_goal_clearance_m: float = 0.22,
    min_goal_distance_m: float = 0.80,
    information_gain_weight: float = 2.0,
    distance_weight: float = 1.0,
) -> FrontierResult:
    """
    Extract, validate and rank frontier clusters.

    A frontier is a known-free cell with at least one four-connected unknown
    neighbour. Clusters use eight-connectivity. Goals must be reachable from
    the robot without entering unknown/occupied cells and must keep the
    requested clearance from occupied cells.
    """
    if min_cluster_size <= 0:
        raise ValueError('min_cluster_size must be positive')
    if min_goal_clearance_m < 0.0 or min_goal_distance_m < 0.0:
        raise ValueError('goal distances must be non-negative')
    if free_threshold < 0 or occupied_threshold <= free_threshold:
        raise ValueError('occupancy thresholds are inconsistent')

    frontiers = _frontier_cells(grid, free_threshold)
    clusters = _cluster_frontiers(grid, frontiers)
    robot_cell = grid.world_to_cell(robot_x, robot_y)
    reachable = _reachable_distances(grid, robot_cell, free_threshold)
    clearance_cells = max(
        0,
        math.ceil(min_goal_clearance_m / grid.resolution),
    )

    rejected_small = 0
    rejected_unreachable = 0
    rejected_near = 0
    rejected_clearance = 0
    candidates = []
    for cluster in clusters:
        if len(cluster) < min_cluster_size:
            rejected_small += 1
            continue
        reachable_cells = [cell for cell in cluster if cell in reachable]
        if not reachable_cells:
            rejected_unreachable += 1
            continue
        distant_cells = [
            cell for cell in reachable_cells
            if reachable[cell] * grid.resolution >= min_goal_distance_m
        ]
        if not distant_cells:
            rejected_near += 1
            continue
        safe_cells = [
            cell for cell in distant_cells
            if _has_occupied_clearance(
                grid,
                cell,
                occupied_threshold,
                clearance_cells,
            )
        ]
        if not safe_cells:
            rejected_clearance += 1
            continue

        goal = max(
            safe_cells,
            key=lambda cell: (
                reachable[cell],
                -cell[0],
                -cell[1],
            ),
        )
        x, y = grid.cell_to_world(goal)
        path_distance = reachable[goal] * grid.resolution
        information_gain = len(cluster) * grid.resolution * grid.resolution
        score = (
            information_gain_weight * math.log1p(len(cluster))
            - distance_weight * path_distance
        )
        candidates.append(FrontierCandidate(
            cells=cluster,
            goal_cell=goal,
            x=x,
            y=y,
            yaw=_goal_yaw(grid, goal, cluster),
            path_distance=path_distance,
            information_gain=information_gain,
            score=score,
        ))

    candidates.sort(
        key=lambda item: (-item.score, item.path_distance, item.goal_cell)
    )
    return FrontierResult(
        candidates=tuple(candidates),
        frontier_cells=tuple(sorted(frontiers)),
        cluster_count=len(clusters),
        rejected_small_clusters=rejected_small,
        rejected_unreachable_clusters=rejected_unreachable,
        rejected_near_clusters=rejected_near,
        rejected_clearance_clusters=rejected_clearance,
    )


def is_exploration_complete(
    *,
    frontier_cell_count: int,
    eligible_candidate_count: int,
    known_area_m2: float,
    max_residual_frontier_cells: int = 40,
    min_known_area_m2: float = 80.0,
) -> bool:
    """
    Decide whether only insignificant, unusable frontiers remain.

    A map with no frontier is complete unconditionally. A small residual set
    is accepted only after high map coverage and only when none of those
    frontiers can produce a reachable, clearance-safe navigation goal.
    """
    values = (
        frontier_cell_count,
        eligible_candidate_count,
        max_residual_frontier_cells,
    )
    if any(value < 0 for value in values):
        raise ValueError('frontier completion counts must be non-negative')
    if known_area_m2 < 0.0 or min_known_area_m2 < 0.0:
        raise ValueError('frontier completion areas must be non-negative')
    if frontier_cell_count == 0:
        return True
    return (
        eligible_candidate_count == 0
        and frontier_cell_count <= max_residual_frontier_cells
        and known_area_m2 >= min_known_area_m2
    )


def save_grid_map(
    grid: GridMap,
    output_stem,
    *,
    free_threshold: int = 20,
    occupied_threshold: int = 65,
) -> tuple[Path, Path]:
    """Write a Nav2-compatible trinary PGM/YAML pair from a grid."""
    if free_threshold < 0 or occupied_threshold <= free_threshold:
        raise ValueError('occupancy thresholds are inconsistent')
    stem = Path(output_stem).expanduser()
    if stem.suffix.lower() in {'.yaml', '.yml', '.pgm'}:
        stem = stem.with_suffix('')
    stem.parent.mkdir(parents=True, exist_ok=True)
    image_path = stem.with_suffix('.pgm')
    yaml_path = stem.with_suffix('.yaml')

    pixels = bytearray()
    for row in range(grid.height - 1, -1, -1):
        for column in range(grid.width):
            value = grid.value((row, column))
            if value < 0:
                pixels.append(205)
            elif value >= occupied_threshold:
                pixels.append(0)
            elif value <= free_threshold:
                pixels.append(254)
            else:
                pixels.append(205)
    header = (
        f'P5\n# CREATOR: tb3_nav_assessment frontier_explorer\n'
        f'{grid.width} {grid.height}\n255\n'
    ).encode('ascii')
    image_path.write_bytes(header + pixels)
    yaml_text = (
        f'image: {image_path.name}\n'
        'mode: trinary\n'
        f'resolution: {grid.resolution:.9g}\n'
        'origin: '
        f'[{grid.origin_x:.9g}, {grid.origin_y:.9g}, '
        f'{grid.origin_yaw:.9g}]\n'
        'negate: 0\n'
        f'occupied_thresh: {occupied_threshold / 100.0:.6g}\n'
        f'free_thresh: {free_threshold / 100.0:.6g}\n'
    )
    yaml_path.write_text(yaml_text, encoding='utf-8')
    return yaml_path, image_path
