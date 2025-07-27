import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon as MplPolygon
from shapely.geometry import Point, LineString
import numpy as np
from environment import get_all_scenes
from lectures.IPVISLazyPRM import lazyPRMVisualize
from lectures.IPVISBasicPRM import basicPRMVisualize
from lectures.IPLazyPRM import LazyPRM
from lectures.IPBasicPRM import BasicPRM
import random
from shapely.geometry import Polygon as ShapelyPolygon
from lectures.IPEnvironment import CollisionChecker
from shapely.ops import unary_union
from collections import deque

class HierarchicalPlanner:
    def __init__(self, checker, limits, obstacles, ax=None,
                 planner_type='lazy', connection_radius=15.0, search_margin=5.0):
        self.checker = checker
        self.limits = limits
        self.obstacles = obstacles
        self.ax = ax
        self.planner_type = planner_type
        self.connection_radius = connection_radius
        self.search_margin = search_margin

        self.planner_config = {
            "initialRoadmapSize": 40,
            "updateRoadmapSize": 20,
            "kNearest": 5,
            "maxIterations": 10,
            "radius": 5.0,
            "numNodes": 100
        }

        self.guards = []
        self.node_id = 0
        self.colors = [(0.5, 0.8, 1.0, 0.4), (0.6, 1.0, 0.6, 0.4), (1.0, 0.6, 0.6, 0.4)]
        self.debug_stats = []
        self.visibility_polygons = []

    def clamp_bounds(self, min_x, max_x, min_y, max_y):
        return (
            max(min_x, self.limits[0][0]),
            min(max_x, self.limits[0][1]),
            max(min_y, self.limits[1][0]),
            min(max_y, self.limits[1][1])
        )

    def compute_visibility_polygon_raycast(self, center, max_radius=25.0, steps=360, step_size=0.5):
        angles = np.linspace(0, 2 * np.pi, steps, endpoint=False)
        visible_points = []
        for angle in angles:
            for dist in np.arange(0.1, max_radius, step_size):
                x = center[0] + np.cos(angle) * dist
                y = center[1] + np.sin(angle) * dist
                test_point = (x, y)
                ray = LineString([center, test_point])
                if any(ob.contains(Point(test_point)) or ob.intersects(ray) for ob in self.obstacles):
                    x = center[0] + np.cos(angle) * (dist - step_size)
                    y = center[1] + np.sin(angle) * (dist - step_size)
                    visible_points.append((x, y))
                    break
            else:
                visible_points.append((x, y))
        return visible_points

    def run_local_planner(self, p1, p2, bounds):
        planner_cls = LazyPRM if self.planner_type == 'lazy' else BasicPRM
        planner = planner_cls(self.checker)
        planner.setSamplingBounds(bounds)
        path = planner.planPath([p1], [p2], self.planner_config)

        self.debug_stats.append({
            'type': self.planner_type,
            'start': p1,
            'goal': p2,
            'success': bool(path),
            'collidingEdges': getattr(planner, 'collidingEdges', []),
            'validEdges': getattr(planner, 'nonCollidingEdges', [])
        })

        return path

    def try_hierarchical_connection(self, current_pos):
        for other_id, other_pos in self.guards:
            if np.allclose(current_pos, other_pos):
                continue
            if np.linalg.norm(np.array(current_pos) - np.array(other_pos)) > self.connection_radius:
                continue

            min_x = min(current_pos[0], other_pos[0]) - self.search_margin
            max_x = max(current_pos[0], other_pos[0]) + self.search_margin
            min_y = min(current_pos[1], other_pos[1]) - self.search_margin
            max_y = max(current_pos[1], other_pos[1]) + self.search_margin
            min_x, max_x, min_y, max_y = self.clamp_bounds(min_x, max_x, min_y, max_y)

            planner_cls = LazyPRM if self.planner_type == 'lazy' else BasicPRM
            local_planner = planner_cls(self.checker)
            local_planner.setSamplingBounds(((min_x, max_x), (min_y, max_y)))

            # 🚀 Lokale Planung
            path = local_planner.planPath([current_pos], [other_pos], self.planner_config)

            self.debug_stats.append({
                'type': self.planner_type,
                'start': current_pos,
                'goal': other_pos,
                'success': bool(path),
                'collidingEdges': getattr(local_planner, 'collidingEdges', []),
                'validEdges': getattr(local_planner, 'nonCollidingEdges', []),
                'nodes': list(local_planner.graph.nodes(data=True)),
                'edges': list(local_planner.graph.edges())
            })

            if self.ax:
                color = 'green' if path else 'orange'
                self.ax.plot(
                    [current_pos[0], other_pos[0]],
                    [current_pos[1], other_pos[1]],
                    linestyle='-' if path else '--',
                    color=color, alpha=0.7, linewidth=1
                )

    def place_guard(self, pos):
        polygon_points = self.compute_visibility_polygon_raycast(pos)
        new_poly = ShapelyPolygon(polygon_points)

        # Repair polygons
        safe_new_poly = new_poly.buffer(0)
        safe_union_prev = unary_union(self.visibility_polygons).buffer(0) if self.visibility_polygons else None

        if safe_union_prev:
            new_area = safe_new_poly.area
            added_area = safe_new_poly.difference(safe_union_prev).area
            if added_area < 0.2 or (added_area / new_area) < 0.07:
                return False

        if self.ax:
            patch = MplPolygon(polygon_points, closed=True, color=self.colors[self.node_id % len(self.colors)])
            self.ax.add_patch(patch)
            self.ax.plot(*pos, 'ko', markersize=3)

        self.visibility_polygons.append(safe_new_poly)
        self.guards.append((self.node_id, pos))
        self.try_hierarchical_connection(pos)
        self.node_id += 1
        return True

    def planPath(self, start, goal, config):
        self.guards = []
        self.node_id = 0
        self.debug_stats = []
        self.visibility_polygons = []

        # Guard Sampling
        placed = 0
        for _ in range(config["ntry"]):
            sample = np.random.uniform(
                low=[self.limits[0][0], self.limits[1][0]],
                high=[self.limits[0][1], self.limits[1][1]]
            )
            if not self.checker.pointInCollision(sample):
                self.place_guard(sample)
                placed += 1

        # Start + Ziel als letzte Guards
        self.place_guard(start)
        self.place_guard(goal)

        # Sichtbarkeitsgraph erzeugen
        graph = {}
        for i, (_, p1) in enumerate(self.guards):
            for j, (_, p2) in enumerate(self.guards):
                if i == j:
                    continue
                if np.linalg.norm(np.array(p1) - np.array(p2)) <= self.connection_radius:
                    graph.setdefault(tuple(p1), []).append(tuple(p2))

        # Path Suche via DFS
        start_pt = tuple(start)
        goal_pt = tuple(goal)
        visited = set()
        stack = [(start_pt, [start_pt])]
        while stack:
            node, path = stack.pop()
            if node == goal_pt:
                return path
            if node in visited:
                continue
            visited.add(node)
            for neighbor in graph.get(node, []):
                stack.append((neighbor, path + [neighbor]))

        return []

    def draw_debug(self, ax):
        for entry in self.debug_stats:
            node_dict = dict(entry.get("nodes", []))

            # Zeichne alle Knoten als Punkte
            for node_data in node_dict.values():
                x, y = node_data['pos']
                ax.plot(x, y, marker='.', color='black', alpha=0.2, markersize=2)

            # Zeichne Kanten (falls vorhanden)
            for (a, b) in entry.get("edges", []):
                if a in node_dict and b in node_dict:
                    p1 = node_dict[a]['pos']
                    p2 = node_dict[b]['pos']
                    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color='gray', alpha=0.3, linewidth=0.5)

