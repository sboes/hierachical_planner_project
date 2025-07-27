import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union
import numpy as np
from environment import get_all_scenes

# --- Szene laden ---
scene_name, (obstacles, limits, _) = get_all_scenes()[0]
obstacle_list = list(obstacles.values())

# --- Plot vorbereiten ---
fig, ax = plt.subplots(figsize=(6, 6))
plt.subplots_adjust(bottom=0.15)
ax.set_xlim(*limits[0])
ax.set_ylim(*limits[1])
ax.set_aspect('equal')
ax.set_title("Sichtfeld-Polygone (Farbig)")

# --- Hindernisse zeichnen ---
for poly in obstacle_list:
    patch = MplPolygon(list(poly.exterior.coords), closed=True, color='gray', alpha=0.5)
    ax.add_patch(patch)

# --- Farben für Sichtflächen ---
color_cycle = [
    (0.5, 0.8, 1.0, 0.4),  # hellblau
    (0.6, 1.0, 0.6, 0.4),  # hellgrün
    (1.0, 0.6, 0.6, 0.4),  # rosa
    (1.0, 1.0, 0.5, 0.4),  # gelblich
    (0.8, 0.6, 1.0, 0.4),  # violett
]
color_index = 0
visibility_polygons = []

# --- Sichtfeldberechnung via Raycasting ---
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
                # Letzter freier Punkt vor Hindernis
                x = center[0] + np.cos(angle) * (dist - step_size)
                y = center[1] + np.sin(angle) * (dist - step_size)
                visible_points.append((x, y))
                break
        else:
            # Kein Hindernis getroffen
            visible_points.append((x, y))
    return visible_points

# --- Klickverhalten ---
def on_click(event):
    global color_index

    if event.inaxes != ax or event.button != 1:
        return

    pos = (event.xdata, event.ydata)
    print(f"→ Klick bei {pos}")

    # 1. Sichtfläche erzeugen
    polygon_points = compute_visibility_polygon_raycast(pos, obstacle_list)
    new_poly = Polygon(polygon_points)

    # 2. Vergleich mit gepuffertem Sichtbereich
    if visibility_polygons:
        union_prev = unary_union(visibility_polygons).buffer(0.01)
        new_area = new_poly.area
        added_area = new_poly.difference(union_prev).area

        print(f"→ Fläche gesamt: {new_area:.3f}, neu sichtbar: {added_area:.3f} ({(added_area/new_area)*100:.2f}%)")

        # 3. Schwellenwertprüfung (absolut & relativ)
        if added_area < 0.2 or (added_area / new_area) < 0.02:
            ax.plot(*pos, marker='x', color='gray', alpha=0.4)
            fig.canvas.draw()
            return

    # 4. Sichtfläche anzeigen
    patch = MplPolygon(polygon_points, closed=True, color=color_cycle[color_index % len(color_cycle)])
    ax.add_patch(patch)
    visibility_polygons.append(new_poly)

    # 5. Punkt markieren
    ax.plot(*pos, 'o', color='black')
    color_index += 1
    fig.canvas.draw()

# --- Event Handler aktivieren ---
fig.canvas.mpl_connect('button_press_event', on_click)
plt.show()
