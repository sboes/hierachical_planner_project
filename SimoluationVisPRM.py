'''import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from shapely.geometry import Point, LineString
import networkx as nx
from environment import get_all_scenes
from lectures.IPVisibilityPRM import VisPRM

# --- Collision Checker ---
class ShapelyCollisionChecker:
    def __init__(self, obstacles):
        self.obstacles = list(obstacles)

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

# --- Setup ---
scene_name, (obstacles, limits, _) = get_all_scenes()[0]
checker = ShapelyCollisionChecker(list(obstacles.values()))
planner = VisPRM(checker)

fig, ax = plt.subplots(figsize=(6, 6))
ax.set_xlim(*limits[0])
ax.set_ylim(*limits[1])
ax.set_aspect('equal')
ax.set_title("Interactive Visibility PRM")
checker.drawObstacles(ax)

# --- Interaktive Klickfunktion ---
def on_click(event):
    if event.inaxes != ax or event.button != 1:
        return

    pos = (float(event.xdata), float(event.ydata))
    if checker.pointInCollision(pos):
        print("✖ Punkt in Kollision – wird ignoriert")
        return

    node_id = len(planner.graph.nodes)
    visible_guards = []

    for node in planner.graph.nodes:
        if planner.graph.nodes[node].get('nodeType') == 'Guard':
            other_pos = planner.graph.nodes[node]['pos']
            if not checker.lineInCollision(pos, other_pos):
                visible_guards.append(node)

    if len(visible_guards) == 0:
        print(f"✔ Neuer Guard {node_id} bei {pos}")
        planner.graph.add_node(node_id, pos=pos, color='red', nodeType='Guard')
    elif len(visible_guards) == 1:
        print(f"✖ Nur ein Guard sichtbar – Punkt {pos} wird verworfen")
        return
    else:
        print(f"🔗 Neuer Connection-Knoten {node_id} bei {pos}, sichtbar von {visible_guards}")
        planner.graph.add_node(node_id, pos=pos, color='blue', nodeType='Connection')
        planner.graph.add_edge(node_id, visible_guards[0])
        planner.graph.add_edge(node_id, visible_guards[1])

    # Zeichnen
    posList = nx.get_node_attributes(planner.graph, 'pos')
    types = nx.get_node_attributes(planner.graph, 'nodeType')
    colors = nx.get_node_attributes(planner.graph, 'color')

    ax.clear()
    checker.drawObstacles(ax)
    ax.set_xlim(*limits[0])
    ax.set_ylim(*limits[1])
    ax.set_aspect('equal')
    ax.set_title("Interactive Visibility PRM")

    for node, p in posList.items():
        c = colors.get(node, 'black')
        t = types.get(node, '')
        if t == 'Guard':
            ax.plot(p[0], p[1], 's', color=c, markersize=6)
        elif t == 'Connection':
            ax.plot(p[0], p[1], 'o', color=c, markersize=5)

    for u, v in planner.graph.edges():
        pu, pv = posList[u], posList[v]
        ax.plot([pu[0], pv[0]], [pu[1], pv[1]], 'k-', alpha=0.5)

    plt.draw()

fig.canvas.mpl_connect('button_press_event', on_click)
plt.show()'''

