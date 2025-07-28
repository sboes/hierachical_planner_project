import matplotlib.pyplot as plt
from matplotlib.widgets import Button, RadioButtons
from matplotlib.patches import Polygon as MplPolygon
from shapely.geometry import Point, LineString
import networkx as nx
import numpy as np

# --- ENVIRONMENT LADEN ---
from environment import get_all_scenes
scene_name, (obstacles_dict, limits, _) = get_all_scenes()[0]
obstacles = list(obstacles_dict.values())

# --- COLLISION CHECKER ---
class ShapelyCollisionChecker:
    def __init__(self, obstacles):
        self.obstacles = obstacles

    def getDim(self):
        return 2

    def getEnvironmentLimits(self):
        min_x = min(ob.bounds[0] for ob in self.obstacles)
        min_y = min(ob.bounds[1] for ob in self.obstacles)
        max_x = max(ob.bounds[2] for ob in self.obstacles)
        max_y = max(ob.bounds[3] for ob in self.obstacles)
        return [(min_x, max_x), (min_y, max_y)]

    def pointInCollision(self, pos):
        return any(ob.contains(Point(pos)) for ob in self.obstacles)

    def lineInCollision(self, p1, p2):
        return any(ob.intersects(LineString([p1, p2])) for ob in self.obstacles)

    def drawObstacles(self, ax):
        for poly in self.obstacles:
            patch = MplPolygon(list(poly.exterior.coords), closed=True, color='gray', alpha=0.5)
            ax.add_patch(patch)

checker = ShapelyCollisionChecker(obstacles)

# --- IMPORTIERE SUBPLANNER UND HIERARCHICAL PRM ---
from lectures.IPLazyPRM import LazyPRM
from lectures.IPBasicPRM import BasicPRM
from planner import HierarchicalPRM

# --- DEFAULT-KONFIG ---
lazy_config = {
    "initialRoadmapSize": 15,
    "updateRoadmapSize": 10,
    "kNearest": 8,
    "maxIterations": 10
}

basic_config = {
    "radius": 5.0,
    "numNodes": 200
}

planner = HierarchicalPRM(checker, LazyPRM, lazy_config)
planner.latest_subplanner = None  # Für Visualisierung
planner.latest_bounds = None      # Für Sichtbarkeit des Suchraums

original_localSubplan = planner._localSubplan

def patched_localSubplan(start, goal):
    margin = 0.5
    min_x = min(start[0], goal[0]) - margin
    max_x = max(start[0], goal[0]) + margin
    min_y = min(start[1], goal[1]) - margin
    max_y = max(start[1], goal[1]) + margin

    planner.latest_bounds = [(min_x, max_x), (min_y, max_y)]
    print(f"📦 Suchraum für Subplanner: x=[{min_x}, {max_x}], y=[{min_y}, {max_y}]")

    old_limits = planner._collisionChecker.getEnvironmentLimits
    planner._collisionChecker.getEnvironmentLimits = lambda: [(min_x, max_x), (min_y, max_y)]

    subplanner = planner.subplanner_class(planner._collisionChecker)
    subplanner.planPath([start], [goal], planner.subplanner_config)
    planner.latest_subplanner = subplanner

    planner._collisionChecker.getEnvironmentLimits = old_limits

    try:
        path = nx.shortest_path(subplanner.graph, "start", "goal")
        return [subplanner.graph.nodes[n]['pos'] for n in path]
    except:
        return None

planner._localSubplan = patched_localSubplan

# --- VISUALISIERUNG ---
fig, ax = plt.subplots(figsize=(8, 8))
plt.subplots_adjust(bottom=0.3)
ax.set_xlim(*limits[0])
ax.set_ylim(*limits[1])
ax.set_aspect('equal')
ax.set_title("Hierarchical PRM (click to add guards)")
checker.drawObstacles(ax)

solution_path = []
def draw():
    ax.clear()
    checker.drawObstacles(ax)
    ax.set_xlim(*limits[0])
    ax.set_ylim(*limits[1])
    ax.set_aspect('equal')
    ax.set_title("Hierarchical PRM (click to add guards)")

    pos = nx.get_node_attributes(planner.graph, 'pos')
    color = nx.get_node_attributes(planner.graph, 'color')
    nx.draw_networkx_nodes(planner.graph, pos, node_color=list(color.values()), node_size=100, ax=ax)
    nx.draw_networkx_edges(planner.graph, pos, ax=ax)

    for path in planner.subplanner_results.values():
        for i in range(len(path) - 1):
            x0, y0 = path[i]
            x1, y1 = path[i + 1]
            ax.plot([x0, x1], [y0, y1], 'g--', linewidth=2, alpha=0.7)

    if planner.latest_subplanner:
        sub_pos = nx.get_node_attributes(planner.latest_subplanner.graph, 'pos')
        nx.draw_networkx_nodes(planner.latest_subplanner.graph, sub_pos, node_size=20, node_color='blue', ax=ax)
        for u, v in planner.latest_subplanner.graph.edges():
            if u in sub_pos and v in sub_pos:
                p1 = sub_pos[u]
                p2 = sub_pos[v]
                ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color='blue', linestyle='--', alpha=0.5)

    if planner.latest_bounds:
        (min_x, max_x), (min_y, max_y) = planner.latest_bounds
        ax.plot([min_x, max_x, max_x, min_x, min_x],
                [min_y, min_y, max_y, max_y, min_y],
                color='orange', linestyle='dotted', linewidth=2)

    if solution_path:
        for i in range(len(solution_path) - 1):
            p0 = solution_path[i]
            p1 = solution_path[i + 1]
            pos0 = planner.graph.nodes[p0]['pos'] if isinstance(p0, str) else p0
            pos1 = planner.graph.nodes[p1]['pos'] if isinstance(p1, str) else p1
            ax.plot([pos0[0], pos1[0]], [pos0[1], pos1[1]], 'g-', linewidth=4)

    plt.draw()

def on_click(event):
    if event.inaxes != ax or event.button != 1:
        return
    pos = (float(event.xdata), float(event.ydata))
    if checker.pointInCollision(pos):
        print("✖ Punkt in Kollision – wird ignoriert")
        return

    for node in planner.guards:
        existing_pos = planner.graph.nodes[node]['pos']
        if planner._isVisible(pos, existing_pos):
            print("🚫 Sichtverbindung zu bestehendem Guard – kein neuer Guard platziert.")
            return

    node_id = len(planner.graph.nodes)
    planner.graph.add_node(node_id, pos=pos, color='red', nodeType='Guard')
    planner.guards.append(node_id)
    print(f"✔ Neuer Guard {node_id} bei {pos}")
    draw()

connection_index = 0
def on_step(event):
    global connection_index
    guards = planner.guards
    if connection_index >= len(guards):
        print("✅ Alle Verbindungen getestet.")
        return
    g = guards[connection_index]
    pos_g = planner.graph.nodes[g]['pos']
    distances = [
        (other, np.linalg.norm(np.array(pos_g) - np.array(planner.graph.nodes[other]['pos'])))
        for other in guards if other != g
    ]

    distances.sort(key=lambda x: x[1])
    connections_made = 0
    attempts = 0
    for candidate, _ in distances:
        if connections_made >= 2 or attempts >= 3:
            break
        attempts += 1
        if planner._isVisible(pos_g, planner.graph.nodes[candidate]['pos']):
            planner.graph.add_edge(g, candidate)
            connections_made += 1
        else:
            path = planner._localSubplan(pos_g, planner.graph.nodes[candidate]['pos'])
            if path and len(path) > 1:
                planner.subplanner_results[(g, candidate)] = path
                planner.graph.add_edge(g, candidate)
                connections_made += 1
    connection_index += 1
    draw()

def switch_subplanner(label):
    if label == 'Lazy':
        planner.subplanner_class = LazyPRM
        planner.subplanner_config = lazy_config
    else:
        planner.subplanner_class = BasicPRM
        planner.subplanner_config = basic_config
    print(f"🔁 Subplanner switched to: {label}")

def find_solution(event):
    global solution_path
    start = (1.0, 1.0)
    goal = (21.0, 21.0)

    def connect_visible(pos, label):
        for g in planner.guards:
            guard_pos = planner.graph.nodes[g]['pos']
            if planner._isVisible(pos, guard_pos):
                planner.graph.add_node(label, pos=pos, color='lightgreen')
                planner.graph.add_edge(label, g)
                return g
        return None

    start_guard = connect_visible(start, "start")
    goal_guard = connect_visible(goal, "goal")

    if start_guard is None or goal_guard is None:
        print("❌ Start oder Ziel konnte nicht mit einem Guard verbunden werden.")
        return

    subgraph = nx.Graph()
    for (g1, g2), path in planner.subplanner_results.items():
        subgraph.add_node(g1)
        subgraph.add_node(g2)
        subgraph.add_edge(g1, g2)

    try:
        guard_path = nx.shortest_path(subgraph, start_guard, goal_guard)
    except nx.NetworkXNoPath:
        print("❌ Kein Pfad über Subplanner-Pfade zwischen Guards.")
        return

    full_path = ["start"]
    for i in range(len(guard_path) - 1):
        path_segment = planner.subplanner_results.get((guard_path[i], guard_path[i + 1]))
        if not path_segment:
            path_segment = list(reversed(planner.subplanner_results.get((guard_path[i + 1], guard_path[i]), [])))
        full_path.extend(path_segment)
    full_path.append("goal")

    solution_path.clear()
    solution_path.extend(full_path)
    print(f"✅ Gültiger Pfad gefunden mit {len(full_path)} Punkten.")
    draw()

btn_ax = plt.axes([0.7, 0.05, 0.1, 0.075])
step_btn = Button(btn_ax, 'Step')
step_btn.on_clicked(on_step)

radio_ax = plt.axes([0.05, 0.05, 0.15, 0.15])
radio = RadioButtons(radio_ax, ('Lazy', 'Basic'))
radio.on_clicked(switch_subplanner)

solve_ax = plt.axes([0.82, 0.05, 0.12, 0.075])
solve_btn = Button(solve_ax, 'Find Solution')
solve_btn.on_clicked(find_solution)

fig.canvas.mpl_connect('button_press_event', on_click)
plt.show()