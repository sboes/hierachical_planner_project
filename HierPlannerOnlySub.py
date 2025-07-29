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
        self.points = []   # statt Guards jetzt generische Punkte
        self.latest_subplanner = None
        self.latest_bounds = None

    def _localSubplan(self, start, goal, bounds=None):
        subplanner = self.subplanner_class(self._collisionChecker)

        # SamplingBounds setzen (lokales Rechteck um start)
        if bounds is not None:
            if hasattr(subplanner, 'setSamplingBounds'):
                subplanner.setSamplingBounds(bounds)
            self.latest_bounds = bounds
        else:
            if hasattr(subplanner, 'setSamplingBounds'):
                subplanner.setSamplingBounds(self._collisionChecker.limits)

        subplanner.planPath([start], [goal], self.subplanner_config)
        self.latest_subplanner = subplanner

        if not ("start" in subplanner.graph and "goal" in subplanner.graph):
            return None

        try:
            path = nx.shortest_path(subplanner.graph, "start", "goal")
            path_coords = [subplanner.graph.nodes[n]['pos'] for n in path]
            return path_coords
        except nx.NetworkXNoPath:
            return None

    @IPPerfMonitor
    def _learnRoadmap(self, ntry=None):
        self.graph.clear()
        self.points = []
        self.subplanner_results.clear()

        ntry = ntry or 30
        attempts = 0
        max_attempts = ntry * 10

        while len(self.points) < ntry and attempts < max_attempts:
            attempts += 1
            sample = [
                np.random.uniform(low, high)
                for (low, high) in self._collisionChecker.limits
            ]

            if self._collisionChecker.pointInCollision(sample):
                continue

            node_id = len(self.graph.nodes)
            self.graph.add_node(node_id, pos=sample, color='red', nodeType='Point')

            # Lokales Rechteck um diesen Punkt (Radius 5)
            margin = 5.0
            (min_x, max_x), (min_y, max_y) = self._collisionChecker.limits
            bounds = [
                (max(sample[0] - margin, min_x), min(sample[0] + margin, max_x)),
                (max(sample[1] - margin, min_y), min(sample[1] + margin, max_y)),
            ]

            connections = 0
            for other in self.points:
                pos_other = self.graph.nodes[other]['pos']
                path = self._localSubplan(sample, pos_other, bounds=bounds)
                if path and len(path) > 1:
                    self.subplanner_results[(node_id, other)] = path
                    self.graph.add_edge(node_id, other)
                    connections += 1

            # Falls mehr als 4 Verbindungen → Punkt verwerfen
            if connections > 4:
                self.graph.remove_node(node_id)
                continue

            self.points.append(node_id)

        print(f"🛠 {len(self.points)} Punkte erzeugt (von {ntry} angefragt)")

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

        # Suche nach nächstgelegenen erreichbaren Punkten
        def connect_to_points(pos, label):
            for p in self.points:
                path = self._localSubplan(pos, self.graph.nodes[p]['pos'])
                if path:
                    self.subplanner_results[(label, p)] = path
                    self.graph.add_edge(label, p)
                    return p
            return None

        start_p = connect_to_points(start_pos, "start")
        goal_p = connect_to_points(goal_pos, "goal")

        if start_p is None or goal_p is None:
            print("❌ Start oder Ziel konnte nicht verbunden werden.")
            return []

        try:
            full_path = nx.shortest_path(self.graph, "start", "goal")
            return [self.graph.nodes[n]['pos'] for n in full_path]
        except nx.NetworkXNoPath:
            print("❌ Kein globaler Pfad gefunden.")
            return []
