import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union
import numpy as np
from environment import get_all_scenes
from lectures.IPLazyPRM import LazyPRM
import networkx as nx

# --- Dummy Collision Checker ---
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

# --- Verbindung mit Hierarchischem PRM ---
def try_connect_hierarchical(p1, p2, checker, subplanner, prm_config, ax_graph):
    if not checker.lineInCollision(p1, p2):
        ax_graph.plot([p1[0], p2[0]], [p1[1], p2[1]], color='green', linewidth=1.5, alpha=0.8)
        print("✔ Direkte Sichtverbindung möglich")
        return True

    print("✖ Keine Sichtverbindung, versuche LazyPRM...")

    subplanner.graph.clear()
    if hasattr(subplanner, 'lastGeneratedNodeNumber'):
        subplanner.lastGeneratedNodeNumber = 0
    if hasattr(subplanner, 'collidingEdges'):
        subplanner.collidingEdges.clear()

    path = subplanner.planPath([p1], [p2], prm_config)

    if path and len(path) >= 2:
        pos_map = nx.get_node_attributes(subplanner.graph, 'pos')
        path_coords = [pos_map[n] for n in path if n in pos_map]
        if len(path_coords) >= 2:
            xs, ys = zip(*path_coords)
            ax_graph.plot(xs, ys, color='orange', linewidth=1.5, alpha=0.8)
            print("✔ Verbindung über Subplaner hergestellt")
            return True
    else:
        ax_graph.plot([p1[0], p2[0]], [p1[1], p2[1]], color='gray', linestyle='dashed', alpha=0.3)
        print("✖ Verbindung nicht möglich")

    return False

# --- Sichtfeldberechnung ---
def compute_visibility_polygon_raycast(center, obstacles, max_radius=25.0, steps=360, step_size=0.5):
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

# --- Szene laden ---
scene_name, (obstacles, limits, _) = get_all_scenes()[0]
obstacle_list = list(obstacles.values())
checker = ShapelyCollisionChecker(obstacle_list)
subplanner = LazyPRM(checker)

# --- Plot vorbereiten ---
fig, ax = plt.subplots(figsize=(6, 6))
plt.subplots_adjust(bottom=0.15)
ax.set_xlim(*limits[0])
ax.set_ylim(*limits[1])
ax.set_aspect('equal')
ax.set_title("Hierarchical Visibility + LazyPRM")
checker.drawObstacles(ax)

# --- Farben ---
color_cycle = [
    (0.5, 0.8, 1.0, 0.4),
    (0.6, 1.0, 0.6, 0.4),
    (1.0, 0.6, 0.6, 0.4),
    (1.0, 1.0, 0.5, 0.4),
    (0.8, 0.6, 1.0, 0.4),
]
color_index = 0
visibility_polygons = []
guards = []
node_id = 1

# --- Klickverhalten ---
def on_click(event):
    global color_index, node_id

    if event.inaxes != ax or event.button != 1:
        return

    pos = (event.xdata, event.ydata)
    print(f"\n→ Klick bei {pos}")

    polygon_points = compute_visibility_polygon_raycast(pos, obstacle_list)
    new_poly = Polygon(polygon_points)

    if visibility_polygons:
        union_prev = unary_union(visibility_polygons).buffer(0.01)
        new_area = new_poly.area
        added_area = new_poly.difference(union_prev).area

        print(f"→ Fläche: total={new_area:.2f}, neu={added_area:.2f}, Anteil={(added_area/new_area)*100:.2f}%")

        if added_area < 0.2 or (added_area / new_area) < 0.07:
            ax.plot(*pos, marker='x', color='gray', alpha=0.4)
            fig.canvas.draw()
            return

    patch = MplPolygon(polygon_points, closed=True, color=color_cycle[color_index % len(color_cycle)])
    ax.add_patch(patch)
    visibility_polygons.append(new_poly)

    ax.plot(*pos, 'o', color='black')
    guards.append((node_id, pos))
    color_index += 1

    # --- Hierarchisches Verbinden ---
    connection_radius = 15.0
    for other_id, other_pos in guards:
        if other_id == node_id:
            continue
        dist = np.linalg.norm(np.array(pos) - np.array(other_pos))
        if dist <= connection_radius:
            print(f"→ Verbindungsversuch: {node_id} ↔ {other_id} (Distanz: {dist:.2f})")
            try_connect_hierarchical(
                pos, other_pos, checker, subplanner,
                {
                    "initialRoadmapSize": 40,
                    "updateRoadmapSize": 20,
                    "kNearest": 5,
                    "maxIterations": 10
                },
                ax
            )

    node_id += 1
    fig.canvas.draw()

# --- Interaktion aktivieren ---
fig.canvas.mpl_connect('button_press_event', on_click)
plt.show()
