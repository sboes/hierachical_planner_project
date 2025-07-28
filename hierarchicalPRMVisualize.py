import networkx as nx

def hierarchicalPRMVisualize(planner, solution=[], ax=None, nodeSize=300):
    graph = planner.graph.copy()
    pos = nx.get_node_attributes(graph, 'pos')
    color = nx.get_node_attributes(graph, 'color')
    collChecker = planner._collisionChecker

    # Hindernisse zeichnen
    collChecker.drawObstacles(ax, inWorkspace=True)

    # Knoten zeichnen
    nx.draw_networkx_nodes(graph, pos, ax=ax, nodelist=list(color.keys()), node_color=list(color.values()), node_size=nodeSize)

    # Alle Kanten (grau)
    nx.draw_networkx_edges(graph, pos, ax=ax, edge_color='lightgray', width=1)

    # Subplanner-Ergebnisse (grün gestrichelt)
    for subpath in planner.subplanner_results.values():
        for i in range(len(subpath) - 1):
            p0, p1 = subpath[i], subpath[i + 1]
            ax.plot([p0[0], p1[0]], [p0[1], p1[1]], linestyle='--', color='green', linewidth=2, alpha=0.6)

    # Start- und Zielknoten
    if "start" in graph.nodes:
        nx.draw_networkx_nodes(graph, pos, nodelist=["start"], node_size=nodeSize, node_color="#00dd00", ax=ax)
        nx.draw_networkx_labels(graph, pos, labels={"start": "S"}, ax=ax)
    if "goal" in graph.nodes:
        nx.draw_networkx_nodes(graph, pos, nodelist=["goal"], node_size=nodeSize, node_color="#dd0000", ax=ax)
        nx.draw_networkx_labels(graph, pos, labels={"goal": "G"}, ax=ax)

    # Finaler Pfad (grün durchgezogen)
    if solution:
        Gsp = nx.subgraph(graph, solution)
        nx.draw_networkx_edges(Gsp, pos, alpha=0.9, edge_color='darkgreen', width=4)

    return
