import matplotlib.pyplot as plt
from matplotlib.widgets import Button, RadioButtons
from matplotlib.patches import Polygon as MplPolygon, Rectangle
import networkx as nx
import numpy as np

from environment import get_all_scenes
from HierPlannerOnlySub import HierarchicalPRM
from lectures.IPLazyPRM import LazyPRM
from lectures.IPBasicPRM import BasicPRM

# --- ENVIRONMENT LADEN ---
scene_name, (obstacles_dict, limits, _) = get_all_scenes()[0]
obstacles = list(obstacles_dict.values())

# --- COLLISION CHECKER ---
from shapely.geometry import Point, LineString

class ShapelyCollisionChecker:
    def __init__(self, obstacles):
        self.obstacles = obstacles
        self.limits = [
            (min(ob.bounds[0] for ob in self.obstacles), max(ob.bounds[2] for ob in self.obstacles)),
            (min(ob.bounds[1] for ob in self.obstacles), max(ob.bounds[3] for ob in self.obstacles)),
        ]

    def getEnvironmentLimits(self):
        return self.limits   # <-- WICHTIG für LazyPRM/BasicPRM

    def pointInCollision(self, pos):
        return any(ob.contains(Point(pos)) for ob in self.obstacles)

    def lineInCollision(self, p1, p2):
        return any(ob.intersects(LineString([p1, p2])) for ob in self.obstacles)

    def drawObstacles(self, ax):
        for poly in self.obstacles:
            patch = MplPolygon(list(poly.exterior.coords), closed=True, color='gray', alpha=0.5)
            ax.add_patch(patch)


checker = ShapelyCollisionChecker(obstacles)

# --- KONFIGURATION ---
lazy_config = {"initialRoadmapSize": 15, "updateRoadmapSize": 10, "kNearest": 8, "maxIterations": 10}
basic_config = {"radius": 5.0, "numNodes": 200}

planner = HierarchicalPRM(checker, LazyPRM, lazy_config)

# --- VISUALISIERUNG ---
fig, ax = plt.subplots(figsize=(8, 8))
plt.subplots_adjust(bottom=0.3)
ax.set_xlim(*limits[0])
ax.set_ylim(*limits[1])
ax.set_aspect('equal')
ax.set_title("Hierarchical PRM mit lokalen Subplanner-Rechtecken")
checker.drawObstacles(ax)

solution_path = []
local_regions = []  # Liste der Bounding-Boxen

def draw():
    ax.clear()
    checker.drawObstacles(ax)
    ax.set_xlim(*limits[0])
    ax.set_ylim(*limits[1])
    ax.set_aspect('equal')
    ax.set_title("Hierarchical PRM mit lokalen Subplanner-Rechtecken")

    pos = nx.get_node_attributes(planner.graph, 'pos')
    color = nx.get_node_attributes(planner.graph, 'color')

    # Default-Farbe für Nodes ohne 'color'
    node_colors = []
    for n in planner.graph.nodes():
        if n in color:
            node_colors.append(color[n])
        else:
            node_colors.append("blue")  # fallback

    nx.draw_networkx_nodes(planner.graph, pos, node_color=node_colors, node_size=100, ax=ax)
    nx.draw_networkx_edges(planner.graph, pos, ax=ax)

    # lokale Subplaner-Rechtecke zeichnen
    for (x, y, w, h) in local_regions:
        rect = Rectangle((x, y), w, h, linewidth=2, edgecolor="blue", facecolor="none", linestyle="--")
        ax.add_patch(rect)

    if solution_path:
        for i in range(len(solution_path) - 1):
            p0, p1 = solution_path[i], solution_path[i + 1]
            ax.plot([p0[0], p1[0]], [p0[1], p1[1]], 'g-', linewidth=3)

    plt.draw()


def onclick(event):
    if event.inaxes != ax or event.button != 1:
        return

    click_pos = (float(event.xdata), float(event.ydata))
    if checker.pointInCollision(click_pos):
        print("❌ Punkt liegt im Hindernis, wird ignoriert.")
        return

    # --- Environment-Limits beachten ---
    (xmin, xmax), (ymin, ymax) = checker.getEnvironmentLimits()

    # Bounding Box definieren (lokal, z. B. 10x10) + Schnitt mit Umgebung
    w, h = 10, 10
    bounds = (
        (max(xmin, click_pos[0] - w/2), min(xmax, click_pos[0] + w/2)),
        (max(ymin, click_pos[1] - h/2), min(ymax, click_pos[1] + h/2))
    )

    # --- Alten Subplanner entfernen (nur isolierte Sub-Knoten löschen) ---
    to_remove = [n for n, d in planner.graph.nodes(data=True) if d.get("nodeType") == "Sub"]
    planner.graph.remove_nodes_from(to_remove)

    # --- Hauptknoten (Klickpunkt) anlegen ---
    node_id = f"main_{len([n for n, d in planner.graph.nodes(data=True) if d.get('nodeType') == 'Main'])}"
    planner.graph.add_node(node_id, pos=click_pos, color="red", nodeType="Main")

    # --- Lokalen Subplanner erzeugen ---
    if planner.subplanner_class == LazyPRM:
        local_planner = LazyPRM(checker)

        def _getRandomPosition_local():
            for _ in range(100):
                x = np.random.uniform(bounds[0][0], bounds[0][1])
                y = np.random.uniform(bounds[1][0], bounds[1][1])
                if not checker.pointInCollision((x, y)):
                    return [x, y]
            raise RuntimeError("Kein freier Punkt gefunden")

        local_planner._getRandomPosition = _getRandomPosition_local
        local_planner._buildRoadmap(lazy_config["initialRoadmapSize"], lazy_config["kNearest"])

    else:
        local_planner = BasicPRM(checker)
        local_planner.setSamplingBounds(bounds)
        local_planner._learnRoadmapNearestNeighbour(basic_config["radius"], basic_config["numNodes"])

    # --- Subplanner-Knoten in globalen Graph übernehmen ---
    for n, d in local_planner.graph.nodes(data=True):
        if n not in planner.graph:
            d["nodeType"] = "Sub"
            if "color" not in d:
                d["color"] = "blue"
            planner.graph.add_node(n, **d)
    for u, v in local_planner.graph.edges():
        if not planner.graph.has_edge(u, v):
            planner.graph.add_edge(u, v)

    # --- Bounding Box merken (nur aktueller) ---
    local_regions.clear()
    local_regions.append((bounds[0][0], bounds[1][0], bounds[1][1]-bounds[0][0], bounds[1][1]-bounds[0][1]))

    print(f"📍 Neuer Subplanner bei {click_pos} gesetzt (lokal {bounds})")
    draw()


def find_solution(event):
    global solution_path
    start = (1.0, 1.0)
    goal = (21.0, 21.0)
    solution_path = planner.planPath([start], [goal], {"ntry": 40})
    if solution_path:
        print(f"✅ Pfad gefunden mit {len(solution_path)} Punkten.")
    else:
        print("❌ Kein Pfad gefunden.")
    draw()

# --- BUTTONS ---
solve_ax = plt.axes([0.82, 0.05, 0.12, 0.075])
solve_btn = Button(solve_ax, 'Find Solution')
solve_btn.on_clicked(find_solution)

radio_ax = plt.axes([0.05, 0.05, 0.15, 0.15])
radio = RadioButtons(radio_ax, ('Lazy', 'Basic'))
radio.on_clicked(lambda l: setattr(planner, "subplanner_class", LazyPRM if l == "Lazy" else BasicPRM))

# --- Mausklick Event ---
fig.canvas.mpl_connect('button_press_event', onclick)

plt.show()
