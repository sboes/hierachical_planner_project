# coding: utf-8

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
        self.subplanner_results = {}  # Save subplanner paths
        self.guards = []  # External interface for user-defined guards

    def _isVisible(self, pos1, pos2):
        return not self._collisionChecker.lineInCollision(pos1, pos2)

    @IPPerfMonitor
    def _learnRoadmap(self, ntry=None):
        self.graph.clear()
        self.guards = []

        ntry = ntry or 30
        attempts = 0
        max_attempts = ntry * 10

        env_limits = self._collisionChecker.getEnvironmentLimits()
        min_x, max_x = env_limits[0]
        min_y, max_y = env_limits[1]

        # === 1. Guards erzeugen (maximal viele ohne Sichtverbindung) ===
        while len(self.guards) < ntry and attempts < max_attempts:
            attempts += 1
            sample = (
                np.random.uniform(min_x, max_x),
                np.random.uniform(min_y, max_y)
            )

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

        # === 2. Verbindungen über Subplanner oder direkte Sichtbarkeit ===
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

                if self._isVisible(pos_g, pos_c):
                    self.graph.add_edge(g, candidate)
                    connections_made += 1
                else:
                    path = self._localSubplan(pos_g, pos_c)
                    if path:
                        self.subplanner_results[(g, candidate)] = path
                        self.graph.add_edge(g, candidate)
                        connections_made += 1

    def _localSubplan(self, start, goal):
        subplanner = self.subplanner_class(self._collisionChecker)
        env_limits = self._collisionChecker.getEnvironmentLimits()
        min_env_x, max_env_x = env_limits[0]
        min_env_y, max_env_y = env_limits[1]

        # Optionaler Margin-Wert aus Config (None = voller Raum)
        margin = self.subplanner_config.get("margin", None)

        if margin is None:
            # Gesamter Raum als Sampling-Bereich
            min_x, max_x = min_env_x, max_env_x
            min_y, max_y = min_env_y, max_env_y
        else:
            # Sampling-Bereich begrenzt um Start/Ziel mit Margin, aber innerhalb der Limits
            min_x = max(min(start[0], goal[0]) - margin, min_env_x)
            max_x = min(max(start[0], goal[0]) + margin, max_env_x)
            min_y = max(min(start[1], goal[1]) - margin, min_env_y)
            max_y = min(max(start[1], goal[1]) + margin, max_env_y)

        print(f"📦 Subplanner SamplingBounds: x=[{min_x}, {max_x}], y=[{min_y}, {max_y}]")

        # Sampling-Bereich setzen
        if hasattr(subplanner, 'setSamplingBounds'):
            subplanner.setSamplingBounds(((min_x, max_x), (min_y, max_y)))

        # Plane Pfad mit Subplanner
        subplanner.planPath([start], [goal], self.subplanner_config)
        self.latest_subplanner = subplanner

        # Sicherstellen, dass start & goal im Subplanner-Graph vorhanden sind
        if not ("start" in subplanner.graph and "goal" in subplanner.graph):
            print("⚠️ Subplanner hat start oder goal nicht verbunden.")
            return None

        try:
            path = nx.shortest_path(subplanner.graph, "start", "goal")
            return [subplanner.graph.nodes[n]['pos'] for n in path]
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

        # --- Start- und Ziel-Knoten verbinden ---
        posList = nx.get_node_attributes(self.graph, 'pos')
        self.graph.add_node("start", pos=start_pos, color='lightgreen')
        self.graph.add_node("goal", pos=goal_pos, color='lightgreen')

        start_guard = None
        goal_guard = None

        for node_id, pos in posList.items():
            if not start_guard and self._isVisible(start_pos, pos):
                self.graph.add_edge("start", node_id)
                start_guard = node_id
            if not goal_guard and self._isVisible(goal_pos, pos):
                self.graph.add_edge("goal", node_id)
                goal_guard = node_id
            if start_guard and goal_guard:
                break

        if not (start_guard and goal_guard):
            print("❌ Start oder Ziel konnte nicht mit einem Guard verbunden werden.")
            return []

        # --- Pfad im Subgraph der Subplanner-Pfade finden ---
        subgraph = nx.Graph()
        for (g1, g2), path in self.subplanner_results.items():
            subgraph.add_edge(g1, g2)

        try:
            guard_path = nx.shortest_path(subgraph, start_guard, goal_guard)
        except nx.NetworkXNoPath:
            print("❌ Kein Pfad über Subplanner-Pfade zwischen Guards.")
            return []

        # --- Finalen Pfad zusammensetzen ---
        full_path = []

        # --- Sichtpfad von Start zu erstem Guard ---
        start_guard_pos = self.graph.nodes[start_guard]['pos']
        full_path.append(start_pos)
        full_path.append(start_guard_pos)

        # --- Alle Subplanner-Segmente ---
        for i in range(len(guard_path) - 1):
            a, b = guard_path[i], guard_path[i + 1]
            segment = self.subplanner_results.get((a, b)) or list(reversed(self.subplanner_results.get((b, a), [])))
            if not segment:
                print(f"⚠ Kein Segment zwischen {a} und {b} gefunden.")
                return []
            full_path.extend(segment[1:])  # skip erster Punkt, da identisch

        # --- Sichtpfad vom letzten Guard zum Ziel ---
        goal_guard_pos = self.graph.nodes[goal_guard]['pos']
        full_path.append(goal_guard_pos)
        full_path.append(goal_pos)

        return full_path

