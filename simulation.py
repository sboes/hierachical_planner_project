import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.widgets import Button
from shapely.geometry import Point, LineString
from environment import get_all_scenes
from HierarchicalPRM import HierarchicalPRM  # Deine Klasse!
from lectures.IPLazyPRM import LazyPRM
import networkx as nx

# --- Subplaner-Konfiguration ---
subplanner_config = {
    "initialRoadmapSize": 40,
    "updateRoadmapSize": 20,
    "kNearest": 5,
    "maxIterations": 10
}

# --- Dummy-CollisionChecker ---
class ShapelyCollisionChecker:
    def __init__(self, obstacles):
        self.obstacles = list(obstacles.values())

    def getDim(self):
        return 2

    def pointInCollision(self, pos):
        return any(ob.contains(Point(pos)) for ob in self.obstacles)

    def lineInCollision(self, p1, p2):
        return any(ob.intersects(LineString([p1, p2])) for ob in self.obstacles)

# --- Environment laden ---
scene_name, (obstacles, limits, _) = get_all_scenes()[0]
checker = ShapelyCollisionChecker(obstacles)
planner = HierarchicalPRM(checker, LazyPRM(checker))

# --- Matplotlib vorbereiten ---
fig, ax = plt.subplots(figsize=(6, 6))
plt.subplots_adjust(bottom=0.2)
ax.set_xlim(*limits[0])
ax.set_ylim(*limits[1])
ax.set_aspect('equal')
ax.set_title("Hierarchical PRM mit Lazy Subplaner")

# --- Hindernisse zeichnen ---
for poly in obstacles.values():
    patch = MplPolygon(list(poly.exterior.coords), closed=True, color='gray', alpha=0.5)
    ax.add_patch(patch)

# --- Zustände speichern ---
nodes = []
node_artists = []
lines = []
node_id = 1

# --- Klick-Interaktion ---
def on_click(event):
    global node_id
    if event.inaxes != ax or event.button != 1:
        return

    pos = (event.xdata, event.ydata)
    print(f"\n→ Click at {pos}")

    result, connected = planner.learnSinglePoint(pos, node_id=node_id, config=subplanner_config)

    color_map = {
        "guard": "red",
        "connection": "blue",
        "rejected": "gray"
    }

    # Punkt einzeichnen
    if result == "rejected":
        artist = ax.plot(*pos, 'o', color=color_map[result], alpha=0.3)[0]
    else:
        artist = ax.plot(*pos, 'o', color=color_map[result])[0]

    node_artists.append(artist)
    nodes.append((node_id, pos))

    # Verbindungen einzeichnen
    if result == "connection":
        pos_map = nx.get_node_attributes(planner.graph, 'pos')
        for target_id in connected:
            try:
                path_nodes = nx.shortest_path(planner.graph, node_id, target_id)
                path_coords = [pos_map[n] for n in path_nodes]
                xs, ys = zip(*path_coords)
                line = ax.plot(xs, ys, color='orange')[0]
                lines.append(line)
            except nx.NetworkXNoPath:
                continue

    node_id += 1
    fig.canvas.draw()

# --- Zurücksetzen ---
def clear_all(event):
    global nodes, node_artists, lines, node_id
    for artist in node_artists:
        artist.remove()
    for line in lines:
        line.remove()
    nodes.clear()
    node_artists.clear()
    lines.clear()
    planner.graph.clear()
    planner.edgePaths.clear()
    node_id = 1
    fig.canvas.draw()

# --- Button ---
ax_clear = plt.axes([0.4, 0.05, 0.2, 0.075])
btn_clear = Button(ax_clear, 'Clear All')
btn_clear.on_clicked(clear_all)

fig.canvas.mpl_connect('button_press_event', on_click)
plt.show()
