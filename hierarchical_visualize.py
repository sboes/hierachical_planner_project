import matplotlib.pyplot as plt
import networkx as nx

def visualize_hierarchical_planning(planner, collision_checker, config_low, start, goal):
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    # Setup
    pos_map = nx.get_node_attributes(planner.graph, 'pos')
    xlim, ylim = planner.limits

    # === Plot 1: High-Level Visibility Graph ===
    ax1 = axes[0]
    ax1.set_title("1. Visibility Graph")
    nx.draw_networkx_nodes(planner.graph, pos_map, ax=ax1, node_color='gray', node_size=60)
    nx.draw_networkx_edges(planner.graph, pos_map, ax=ax1, alpha=0.4)
    if 'start' in planner.graph.nodes:
        nx.draw_networkx_nodes(planner.graph, pos_map, nodelist=['start'], node_color='limegreen', node_size=150, ax=ax1)
    if 'goal' in planner.graph.nodes:
        nx.draw_networkx_nodes(planner.graph, pos_map, nodelist=['goal'], node_color='crimson', node_size=150, ax=ax1)
    if hasattr(collision_checker, "drawObstacles"):
        collision_checker.drawObstacles(ax1)

    # === Plot 2: Subplanner-Pfade ===
    ax2 = axes[1]
    ax2.set_title("2. Subplanner-Pfade")
    for (a, b), path in planner.edgePaths.items():
        try:
            if isinstance(path, list) and len(path) > 1 and isinstance(path[0], (tuple, list)):
                xs, ys = zip(*path)
                ax2.plot(xs, ys, linewidth=1.5, alpha=0.7, color="orange")
        except Exception as e:
            print(f"⚠️ Fehler beim Pfad ({a} ↔ {b}): {e}")
    ax2.scatter(*start, color="limegreen", s=100, label="Start")
    ax2.scatter(*goal, color="crimson", s=100, label="Goal")
    if hasattr(collision_checker, "drawObstacles"):
        collision_checker.drawObstacles(ax2)
    ax2.legend()

    # === Plot 3: Finaler Pfad ===
    ax3 = axes[2]
    # --- 3. Finaler Pfad ---
    ax3.set_title("3. Finaler Pfad")

    # 💡 Robuste Filterung gültiger 2D-Koordinaten
    path_coords = [tuple(p) for p in planner.solution_path if isinstance(p, (list, tuple)) and len(p) == 2]

    if path_coords and len(path_coords) >= 2:
        xs, ys = zip(*path_coords)
        ax3.plot(xs, ys, color="blue", linewidth=2.5, label="Pfad")
        ax3.scatter(xs[0], ys[0], color="limegreen", s=100, label="Start")
        ax3.scatter(xs[-1], ys[-1], color="red", s=100, label="Ziel")

    if hasattr(collision_checker, "drawObstacles"):
        collision_checker.drawObstacles(ax3)
    ax3.legend()

    # === Einheitliches Styling für alle ===
    for ax in axes:
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_aspect('equal')
        ax.grid(True)

    plt.tight_layout()
    plt.show()
