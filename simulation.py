import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon as MplPolygon
from shapely.geometry import Point, Polygon
import numpy as np
from environment import get_all_scenes
from lectures.IPVISLazyPRM import lazyPRMVisualize
from lectures.IPVISBasicPRM import basicPRMVisualize
from lectures.IPLazyPRM import LazyPRM
from lectures.IPBasicPRM import BasicPRM
import random
from matplotlib.patches import Polygon as MplPolygon
from shapely.geometry import Point, LineString
from shapely.geometry import Polygon as ShapelyPolygon
from shapely.ops import unary_union


# --- Shapely Collision Checker ---
# --- Shapely Collision Checker ---
class ShapelyCollisionChecker:
    def __init__(self, obstacles):
        self.obstacles = list(obstacles)

    def getDim(self):
        return 2

    def pointInCollision(self, pos):
        return any(ob.contains(Point(pos)) for ob in self.obstacles)

    def lineInCollision(self, p1, p2):
        return any(ob.intersects(LineString([p1, p2])) for ob in self.obstacles)

    def drawObstacles(self, ax):
        for poly in self.obstacles:
            patch = MplPolygon(list(poly.exterior.coords), closed=True, color='gray', alpha=0.5)
            ax.add_patch(patch)

# --- Setup Umgebung ---
scene_name, (obstacles, limits, _) = get_all_scenes()[0]
checker = ShapelyCollisionChecker(list(obstacles.values()))

# --- Parameter ---
connection_radius = 10.0
search_margin = 5.0
planner_type = 'lazy'  # "lazy" oder "basic"
planner_config = {
    "initialRoadmapSize": 40,
    "updateRoadmapSize": 20,
    "kNearest": 5,
    "maxIterations": 10,
    "radius": 5.0,
    "numNodes": 100
}

# --- Initialisierung ---
guards = []
node_id = 0
colors = [(0.5, 0.8, 1.0, 0.4), (0.6, 1.0, 0.6, 0.4), (1.0, 0.6, 0.6, 0.4)]
debug_stats = []
visibility_polygons = []

fig, ax = plt.subplots(figsize=(6, 6))
ax.set_xlim(*limits[0])
ax.set_ylim(*limits[1])
ax.set_aspect('equal')
ax.set_title("Hierarchical Planner (LazyPRM & BasicPRM)")
checker.drawObstacles(ax)

# --- Sichtbarkeitsberechnung ---
def compute_visibility_polygon_raycast(center, obstacles, max_radius=connection_radius, steps=360, step_size=0.5):
    angles = np.linspace(0, 2 * np.pi, steps, endpoint=False)
    visible_points = []
    for angle in angles:
        for dist in np.arange(0.1, max_radius, step_size):
            x = center[0] + np.cos(angle) * dist
            y = center[1] + np.sin(angle) * dist
            test_point = (x, y)
            ray = LineString([center, test_point])
            if any(ob.contains(Point(test_point)) or ob.intersects(ray) for ob in obstacles):
                x = center[0] + np.cos(angle) * (dist - step_size)
                y = center[1] + np.sin(angle) * (dist - step_size)
                visible_points.append((x, y))
                break
        else:
            visible_points.append((x, y))
    return visible_points

# --- Hilfsfunktionen ---
def clamp_bounds(min_x, max_x, min_y, max_y, limits):
    clamped_min_x = max(min_x, limits[0][0])
    clamped_max_x = min(max_x, limits[0][1])
    clamped_min_y = max(min_y, limits[1][0])
    clamped_max_y = min(max_y, limits[1][1])
    return clamped_min_x, clamped_max_x, clamped_min_y, clamped_max_y


def run_local_planner(p1, p2, bounds):
    if planner_type == 'lazy':
        planner = LazyPRM(checker)
        planner.setSamplingBounds(bounds)
        path = planner.planPath([p1], [p2], planner_config)
        lazyPRMVisualize(planner, path, ax=ax)
        num_failures = len(planner.collidingEdges)
        num_success = len(planner.nonCollidingEdges)
        debug_stats.append({
            'type': 'LazyPRM',
            'start': p1,
            'goal': p2,
            'success': path != [],
            'collidingEdges': num_failures,
            'validEdges': num_success
        })
    else:
        planner = BasicPRM(checker)
        planner.setSamplingBounds(bounds)
        path = planner.planPath([p1], [p2], planner_config)
        basicPRMVisualize(planner, path, ax=ax)
        debug_stats.append({
            'type': 'BasicPRM',
            'start': p1,
            'goal': p2,
            'success': path != [],
            'numNodes': planner_config["numNodes"]
        })
    return path


def try_hierarchical_connection(current_pos):
    for other_id, other_pos in guards:
        if np.linalg.norm(np.array(current_pos) - np.array(other_pos)) <= connection_radius:
            if current_pos == other_pos:
                continue

            min_x = min(current_pos[0], other_pos[0]) - search_margin
            max_x = max(current_pos[0], other_pos[0]) + search_margin
            min_y = min(current_pos[1], other_pos[1]) - search_margin
            max_y = max(current_pos[1], other_pos[1]) + search_margin

            min_x, max_x, min_y, max_y = clamp_bounds(min_x, max_x, min_y, max_y, limits)

            # Visualisiere lokale Region
            ax.add_patch(Rectangle((min_x, min_y), max_x - min_x, max_y - min_y,
                                   edgecolor='blue', facecolor='none', linestyle='--', linewidth=1))

            print(f"→ Verbindungsversuch zwischen {current_pos} und {other_pos}")
            path = run_local_planner(current_pos, other_pos, ((min_x, max_x), (min_y, max_y)))
            if path:
                print("✔ Verbindung hergestellt")
            else:
                print("✖ Verbindung fehlgeschlagen")


# --- Interaktive Klickfunktion ---
def on_click(event):
    global node_id
    if event.inaxes != ax or event.button != 1:
        return

    pos = (float(event.xdata), float(event.ydata))
    print(f"\n→ Neuer Guard bei {pos}")

    polygon_points = compute_visibility_polygon_raycast(pos, list(obstacles.values()))
    new_poly = ShapelyPolygon(polygon_points)

    if visibility_polygons:
        union_prev = unary_union(visibility_polygons).buffer(0.01)
        new_area = new_poly.area
        added_area = new_poly.difference(union_prev).area

        print(f"→ Fläche: total={new_area:.2f}, neu={added_area:.2f}, Anteil={(added_area/new_area)*100:.2f}%")

        if added_area < 0.2 or (added_area / new_area) < 0.07:
            ax.plot(*pos, marker='x', color='gray', alpha=0.4)
            fig.canvas.draw()
            return

    # Sichtbarkeitsbereich (als Kreis angedeutet)
    circle = plt.Circle(pos, connection_radius, color=colors[node_id % len(colors)], alpha=0.3)
    ax.add_patch(circle)
    ax.plot(*pos, 'ko')

    visibility_polygons.append(new_poly)
    guards.append((node_id, pos))
    try_hierarchical_connection(pos)

    node_id += 1
    fig.canvas.draw()


fig.canvas.mpl_connect('button_press_event', on_click)
plt.show()

# --- Debug-Stats am Ende ausgeben ---
print("\n--- Debugging Summary ---")
for i, entry in enumerate(debug_stats):
    print(f"[{i+1}] {entry['type']} | Success: {entry['success']} | Start: {entry['start']} → Goal: {entry['goal']}")
    if entry['type'] == 'LazyPRM':
        print(f"    Colliding Edges: {entry['collidingEdges']} | Valid Edges: {entry['validEdges']}")
    else:
        print(f"    Nodes attempted: {entry['numNodes']}")


