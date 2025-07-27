import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon as MplPolygon
from shapely.geometry import Point, Polygon as ShapelyPolygon, LineString
from shapely.ops import unary_union
import numpy as np
import random

from environment import get_all_scenes
from lectures.IPVISLazyPRM import lazyPRMVisualize
from lectures.IPVISBasicPRM import basicPRMVisualize
from lectures.IPLazyPRM import LazyPRM
from lectures.IPBasicPRM import BasicPRM
from matplotlib.widgets import Button

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
        debug_stats.append({
            'type': 'LazyPRM',
            'start': p1,
            'goal': p2,
            'success': path != [],
            'collidingEdges': len(planner.collidingEdges),
            'validEdges': len(planner.nonCollidingEdges)
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

# --- Verbindungslogik mit Rücknahme ---
def process_guard_connection(center_id, center_pos):
    global guards, visibility_polygons

    polygon_points = compute_visibility_polygon_raycast(center_pos, list(obstacles.values()))
    new_poly = ShapelyPolygon(polygon_points)

    if visibility_polygons:
        union_prev = unary_union(visibility_polygons).buffer(0.01)
        new_area = new_poly.area
        added_area = new_poly.difference(union_prev).area
        print(f"→ Fläche: total={new_area:.2f}, neu={added_area:.2f}, Anteil={(added_area / new_area) * 100:.2f}%")
        if added_area < 0.2 or (added_area / new_area) < 0.07:
            print("✖ Guard deckt keine neue Fläche ab – verworfen.")
            return False

    min_x = center_pos[0] - search_margin
    max_x = center_pos[0] + search_margin
    min_y = center_pos[1] - search_margin
    max_y = center_pos[1] + search_margin
    min_x, max_x, min_y, max_y = clamp_bounds(min_x, max_x, min_y, max_y, limits)
    bounds = ((min_x, max_x), (min_y, max_y))

    neighbor_guards = [(gid, pos) for gid, pos in guards if gid != center_id and
                       min_x <= pos[0] <= max_x and min_y <= pos[1] <= max_y]
    if not neighbor_guards:
        if len(guards) == 0:
            print("✔ Kein Nachbar, aber erster Guard – wird behalten")
            visibility_polygons.append(new_poly)
            guards.append((center_id, center_pos))
            circle = plt.Circle(center_pos, connection_radius, color=colors[center_id % len(colors)], alpha=0.3)
            ax.add_patch(circle)
            ax.plot(*center_pos, 'ko')
            return True
        else:
            print("✖ Keine benachbarten Guards im Suchbereich")
            return False

    planner = LazyPRM(checker) if planner_type == 'lazy' else BasicPRM(checker)
    planner.setSamplingBounds(bounds)
    success = False
    valid_paths = []

    for neighbor_id, neighbor_pos in neighbor_guards:
        path = planner.planPath([center_pos], [neighbor_pos], planner_config)

        # Sicherstellen, dass path ein sinnvoller Pfad ist (Liste von Koordinaten)
        if isinstance(path, list) and all(
                isinstance(p, (list, tuple)) and len(p) == 2 and isinstance(p[0], (int, float)) for p in path
        ):
            valid_paths.append((neighbor_id, path))
            success = True

    if success:
        visibility_polygons.append(new_poly)
        guards.append((center_id, center_pos))
        circle = plt.Circle(center_pos, connection_radius, color=colors[center_id % len(colors)], alpha=0.3)
        ax.add_patch(circle)
        ax.plot(*center_pos, 'ko')
        for _, path in valid_paths:
            try:
                x_vals, y_vals = zip(*path)
                ax.plot(x_vals, y_vals, 'b-', linewidth=1.5)
            except Exception as e:
                print(f"⚠ Fehler beim Zeichnen eines Pfads: {e}")
        fig.canvas.draw()  # ← HINZUFÜGEN!
        return True

    else:
        print("✖ Keine gültige Verbindung – Guard verworfen")
        return False


# --- Klick-Interaktion ---
def on_click(event):
    global node_id
    if event.inaxes != ax or event.button != 1:
        return

    pos = (float(event.xdata), float(event.ydata))
    print(f"\n→ Neuer Guard bei {pos}")
    success = process_guard_connection(node_id, pos)
    if success:
        node_id += 1
        fig.canvas.draw()

# --- Button-Funktionen ---
start_point = (limits[0][0] + 1, limits[1][0] + 1)
goal_point = (limits[0][1] - 1, limits[1][1] - 1)

def generate_random_points(event):
    global node_id
    print("\n→ Generiere zufällige Punkte")
    attempts = 0
    while attempts < 20 and node_id < 100:
        x = random.uniform(*limits[0])
        y = random.uniform(*limits[1])
        if checker.pointInCollision((x, y)):
            continue
        success = process_guard_connection(node_id, (x, y))
        if success:
            node_id += 1
        attempts += 1
    fig.canvas.draw()

def find_all_connections(event):
    print("\n→ Finde alle Verbindungen")
    fig.canvas.draw()

def solve_from_start_to_goal(event):
    print("\n→ Starte Pfadsuche von Start zu Ziel")
    ax.plot(*start_point, 'go', label='Start')
    ax.plot(*goal_point, 'ro', label='Ziel')
    bounds = ((min(start_point[0], goal_point[0]), max(start_point[0], goal_point[0])),
              (min(start_point[1], goal_point[1]), max(start_point[1], goal_point[1])))
    path = run_local_planner(start_point, goal_point, bounds)
    if path:
        x_vals, y_vals = zip(*path)
        ax.plot(x_vals, y_vals, 'r-', linewidth=2)
    fig.canvas.draw()

# --- Buttons Setup ---
button_ax1 = plt.axes([0.7, 0.05, 0.25, 0.04])
button_ax2 = plt.axes([0.7, 0.10, 0.25, 0.04])
button_ax3 = plt.axes([0.7, 0.15, 0.25, 0.04])
btn_generate = Button(button_ax1, 'Generate Random Points')
btn_connect = Button(button_ax2, 'Find Connections')
btn_solve = Button(button_ax3, 'Solve Start → Goal')
btn_generate.on_clicked(generate_random_points)
btn_connect.on_clicked(find_all_connections)
btn_solve.on_clicked(solve_from_start_to_goal)

plt.subplots_adjust(bottom=0.25)
fig.canvas.mpl_connect('button_press_event', on_click)
plt.show()

# --- Debug Ausgabe ---
print("\n--- Debugging Summary ---")
for i, entry in enumerate(debug_stats):
    print(f"[{i+1}] {entry['type']} | Success: {entry['success']} | Start: {entry['start']} → Goal: {entry['goal']}")
    if entry['type'] == 'LazyPRM':
        print(f"    Colliding Edges: {entry['collidingEdges']} | Valid Edges: {entry['validEdges']}")
    else:
        print(f"    Nodes attempted: {entry['numNodes']}")
