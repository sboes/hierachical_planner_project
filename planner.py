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
        # 1. Guards werden extern gesetzt (z. B. durch Klicks)
        guards = self.guards

        # 2. Connection Phase (Subplanner pro Region)
        for g in guards:
            pos_g = self.graph.nodes[g]['pos']
            # Find nearest guards to try connection
            distances = [(other, np.linalg.norm(pos_g - self.graph.nodes[other]['pos']))
                         for other in guards if other != g]
            distances.sort(key=lambda x: x[1])

            connections_made = 0
            attempts = 0
            for (candidate, _) in distances:
                if connections_made >= 2 or attempts >= 3:
                    break
                attempts += 1
                if self._isVisible(pos_g, self.graph.nodes[candidate]['pos']):
                    self.graph.add_edge(g, candidate)
                    connections_made += 1
                else:
                    # Fallback: use Subplanner between g and candidate
                    path = self._localSubplan(pos_g, self.graph.nodes[candidate]['pos'])
                    if path:
                        self.subplanner_results[(g, candidate)] = path
                        self.graph.add_edge(g, candidate)
                        connections_made += 1

    def _localSubplan(self, start, goal):
        subplanner = self.subplanner_class(self._collisionChecker)

        # Begrenze den Planungsraum: ein Rechteck, das Start und Ziel enthält, plus etwas Rand
        margin = 2.0
        min_x = min(start[0], goal[0]) - margin
        max_x = max(start[0], goal[0]) + margin
        min_y = min(start[1], goal[1]) - margin
        max_y = max(start[1], goal[1]) + margin

        print(f"📦 Suchraum für Subplanner: x=[{min_x}, {max_x}], y=[{min_y}, {max_y}]")

        # SamplingBounds direkt übergeben – sicherer als nur über das Config-Dict
        if hasattr(subplanner, 'setSamplingBounds'):
            subplanner.setSamplingBounds(((min_x, max_x), (min_y, max_y)))

        # Pfadplanung wie gehabt
        subplanner.planPath([start], [goal], self.subplanner_config)
        self.latest_subplanner = subplanner

        try:
            path = nx.shortest_path(subplanner.graph, "start", "goal")
            return [subplanner.graph.nodes[n]['pos'] for n in path]
        except:
            return None

    @IPPerfMonitor
    def planPath(self, startList, goalList, config):
        self.graph.clear()
        self._learnRoadmap()

        checkedStartList, checkedGoalList = self._checkStartGoal(startList, goalList)
        posList = nx.get_node_attributes(self.graph, 'pos')

        start_connected = False
        for node_id, pos in posList.items():
            if self._isVisible(checkedStartList[0], pos):
                self.graph.add_node("start", pos=checkedStartList[0], color='lightgreen')
                self.graph.add_edge("start", node_id)
                start_connected = True
                break

        goal_connected = False
        for node_id, pos in posList.items():
            if self._isVisible(checkedGoalList[0], pos):
                self.graph.add_node("goal", pos=checkedGoalList[0], color='lightgreen')
                self.graph.add_edge("goal", node_id)
                goal_connected = True
                break

        if not (start_connected and goal_connected):
            print("❌ Start oder Ziel konnte nicht mit der Roadmap verbunden werden.")
            return []

        try:
            path = nx.shortest_path(self.graph, "start", "goal")
        except nx.NetworkXNoPath:
            print("❌ Kein Pfad zwischen Start und Ziel.")
            return []

        return path