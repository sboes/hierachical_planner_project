from lectures.IPPRMBase import PRMBase
import networkx as nx
import numpy as np
from lectures.IPPerfMonitor import IPPerfMonitor

class VisibilityStatsHandler():
    def __init__(self):
        self.graph = nx.Graph()

    def addNodeAtPos(self, nodeNumber, pos):
        self.graph.add_node(nodeNumber, pos=pos, color='yellow')

    def addVisTest(self, fr, to):
        self.graph.add_edge(fr, to)

class HierarchicalPRM(PRMBase):
    def __init__(self, _collChecker, subplanner_class, subplanner_config, _statsHandler=None):
        super(HierarchicalPRM, self).__init__(_collChecker)
        self.graph = nx.Graph()
        self.statsHandler = VisibilityStatsHandler()
        self.subplanner_class = subplanner_class
        self.subplanner_config = subplanner_config
        self.subplanner_results = {}
        self.guards = []

    def _isVisible(self, pos1, pos2):
        return not self._collisionChecker.lineInCollision(pos1, pos2)

    def get_subplan_segment(self, a, b):
        if (a, b) in self.subplanner_results:
            return self.subplanner_results[(a, b)]
        elif (b, a) in self.subplanner_results:
            return list(reversed(self.subplanner_results[(b, a)]))
        return None

    def _find_visible_guard(self, point):
        visible = []
        for node_id in self.guards:
            pos = self.graph.nodes[node_id]['pos']
            if self._isVisible(point, pos):
                dist = np.linalg.norm(np.array(pos) - np.array(point))
                visible.append((dist, node_id))
        if visible:
            visible.sort()
            return visible[0][1]
        return None

    @IPPerfMonitor
    def _learnRoadmap(self, ntry=None):
        self.graph.clear()
        self.guards = []
        self.subplanner_results.clear()

        ntry = ntry or 30
        attempts = 0
        max_attempts = ntry * 10

        # === 1. Guards erzeugen ===
        while len(self.guards) < ntry and attempts < max_attempts:
            attempts += 1
            sample = [
                np.random.uniform(low, high)
                for (low, high) in self._collisionChecker.limits
            ]

            if self._collisionChecker.pointInCollision(sample):
                continue

            is_visible = any(
                self._isVisible(sample, self.graph.nodes[g]['pos'])
                for g in self.guards
            )

            if not is_visible:
                node_id = len(self.graph.nodes)
                self.graph.add_node(node_id, pos=sample, color='red', nodeType='Guard')
                self.guards.append(node_id)

        print(f"🛡 {len(self.guards)} Guards erzeugt (von max. {ntry} angefragt)")

        # === 2. Verbindungen zwischen Guards ===
        for g in self.guards:
            pos_g = self.graph.nodes[g]['pos']
            distances = [
                (other, np.linalg.norm(np.array(pos_g) - np.array(self.graph.nodes[other]['pos'])))
                for other in self.guards if other != g
            ]
            distances.sort(key=lambda x: x[1])

            connections_made = 0
            attempts = 0
            for (candidate, _) in distances:
                if connections_made >= 2 or attempts >= 5:
                    break
                attempts += 1
                pos_c = self.graph.nodes[candidate]['pos']

                key = (g, candidate)
                rev_key = (candidate, g)

                if self.graph.has_edge(g, candidate) or self.graph.has_edge(candidate, g):
                    continue

                if key in self.subplanner_results or rev_key in self.subplanner_results:
                    self.graph.add_edge(g, candidate)
                    connections_made += 1
                    continue

                if self._isVisible(pos_g, pos_c):
                    self.graph.add_edge(g, candidate)
                    connections_made += 1
                else:
                    try:
                        path = self._localSubplan(pos_g, pos_c)
                        if path:
                            self.subplanner_results[key] = path
                            self.graph.add_edge(g, candidate)
                            connections_made += 1
                        else:
                            print(f"⚠️ Subplaner konnte keinen Pfad finden zwischen {g} <-> {candidate}")
                    except Exception as e:
                        print(f"❌ Subplanner Exception für {g} <-> {candidate}: {e}")

    def _localSubplan(self, start, goal):
        subplanner = self.subplanner_class(self._collisionChecker)

        # Wichtig: SamplingBounds auf vollen Konfigurationsraum setzen
        if hasattr(subplanner, 'setSamplingBounds'):
            subplanner.setSamplingBounds(self._collisionChecker.limits)

        print("📦 Subplanner SamplingBounds:", self._collisionChecker.limits)

        subplanner.planPath([start], [goal], self.subplanner_config)
        self.latest_subplanner = subplanner

        if not ("start" in subplanner.graph and "goal" in subplanner.graph):
            print("⚠️ Subplanner hat start oder goal nicht verbunden.")
            return None

        try:
            path = nx.shortest_path(subplanner.graph, "start", "goal")
            path_coords = [subplanner.graph.nodes[n]['pos'] for n in path]

            for i in range(len(path_coords) - 1):
                p1 = path_coords[i]
                p2 = path_coords[i + 1]
                if self._collisionChecker.lineInCollision(p1, p2):
                    print("❌ Subpfad kollidiert mit Objekt.")
                    return None

            return path_coords
        except nx.NetworkXNoPath:
            print("❌ Kein Pfad im Subplanner gefunden.")
            return None

    @IPPerfMonitor
    def planPath(self, startList, goalList, config):
        self.graph.clear()
        self.subplanner_results.clear()
        self._learnRoadmap(ntry=config.get("ntry", 30))

        checkedStartList, checkedGoalList = self._checkStartGoal(startList, goalList)
        start_pos = checkedStartList[0]
        goal_pos = checkedGoalList[0]

        self.graph.add_node("start", pos=start_pos, color='lightgreen')
        self.graph.add_node("goal", pos=goal_pos, color='lightgreen')

        start_guard = self._find_visible_guard(start_pos)
        goal_guard = self._find_visible_guard(goal_pos)

        if start_guard is not None:
            self.graph.add_edge("start", start_guard)
        if goal_guard is not None:
            self.graph.add_edge("goal", goal_guard)

        print(f"Start-Guard: {start_guard}, Goal-Guard: {goal_guard}")

        if start_guard is None or goal_guard is None:
            print("❌ Start oder Ziel konnte nicht mit einem Guard verbunden werden.")
            return []

        # Erstelle einen neuen Graphen mit allen Guards als Knoten
        subgraph = nx.Graph()
        for node_id in self.guards:
            subgraph.add_node(node_id, **self.graph.nodes[node_id])

        # Füge die Kanten aus den Subplanner-Ergebnissen hinzu
        for (g1, g2), path in self.subplanner_results.items():
            if g1 in self.guards and g2 in self.guards:
                subgraph.add_edge(g1, g2)

        try:
            guard_path = nx.shortest_path(subgraph, start_guard, goal_guard)
        except nx.NetworkXNoPath:
            print("❌ Kein Pfad über Subplanner-Pfade zwischen Guards.")
            return []

        full_path = [start_pos, self.graph.nodes[start_guard]['pos']]

        for i in range(len(guard_path) - 1):
            a, b = guard_path[i], guard_path[i + 1]
            segment = self.get_subplan_segment(a, b)
            if not segment:
                print(f"⚠ Kein Segment zwischen {a} und {b} gefunden.")
                return []
            full_path.extend(segment[1:])

        full_path.append(self.graph.nodes[goal_guard]['pos'])
        full_path.append(goal_pos)

        return full_path
