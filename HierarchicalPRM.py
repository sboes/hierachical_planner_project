from lectures.IPPRMBase import PRMBase
import networkx as nx
from scipy.spatial import cKDTree
from lectures.IPPerfMonitor import IPPerfMonitor

class HierarchicalPRM(PRMBase):
    """Hierarchical PRM with a top-level visibility roadmap and sub-level planners for path validation."""

    def __init__(self, _collChecker, subPlanner, limits=None):
        super(HierarchicalPRM, self).__init__(_collChecker)
        self.graph = nx.Graph()
        self.subPlanner = subPlanner  # Must implement planPath(startList, goalList, config)
        self.edgePaths = dict()       # Stores sub-paths for each edge
        self.solution_path = []       # Final path from start to goal
        self.limits = limits          # For visualization scaling

    def _isVisible(self, pos1, pos2, config):
        """Check visibility via sub-planner instead of direct collision check."""
        self.subPlanner.graph.clear()
        if hasattr(self.subPlanner, 'lastGeneratedNodeNumber'):
            self.subPlanner.lastGeneratedNodeNumber = 0
        if hasattr(self.subPlanner, 'collidingEdges'):
            self.subPlanner.collidingEdges.clear()

        path = self.subPlanner.planPath([pos1], [pos2], config)

        if path and len(path) >= 2:
            self.edgePaths[(tuple(pos1), tuple(pos2))] = path
            return True
        return False

    @IPPerfMonitor
    def _learnRoadmap(self, ntry, subConfig):
        nodeNumber = 0
        currTry = 0

        while currTry < ntry:
            q_pos = self._getRandomFreePosition()
            g_vis = None
            merged = False

            for comp in nx.connected_components(self.graph):
                found = False
                for g in comp:
                    if self.graph.nodes[g]['nodeType'] == 'Guard':
                        if self._isVisible(q_pos, self.graph.nodes[g]['pos'], subConfig):
                            found = True
                            if g_vis is None:
                                g_vis = g
                            else:
                                self.graph.add_node(nodeNumber, pos=q_pos, color='lightblue', nodeType='Connection')
                                self.graph.add_edge(nodeNumber, g)
                                self.graph.add_edge(nodeNumber, g_vis)
                                merged = True
                            if found:
                                break
                if merged:
                    break

            if not merged and g_vis is None:
                self.graph.add_node(nodeNumber, pos=q_pos, color='red', nodeType='Guard')
                currTry = 0
            else:
                currTry += 1

            nodeNumber += 1

    @IPPerfMonitor
    def planPath(self, startList, goalList, config):
        self.graph.clear()
        self.edgePaths.clear()
        self.solution_path = []  # <- reset before planning

        subConfig = config.get("subPlannerConfig", {})
        ntry = config.get("ntry", 40)

        checkedStartList, checkedGoalList = self._checkStartGoal(startList, goalList)
        self._learnRoadmap(ntry, subConfig)

        posList = nx.get_node_attributes(self.graph, 'pos')
        kdTree = cKDTree(list(posList.values()))

        def connect_point(label, point):
            if len(posList) == 0:
                return False

            k = min(5, len(posList))
            distances, indices = kdTree.query(point, k=k)

            # Wenn k == 1, dann ist indices ein einzelner int → in Liste umwandeln
            if k == 1:
                indices = [indices]

            for idx in indices:
                node_id = list(posList.keys())[idx]
                if self._isVisible(point, self.graph.nodes[node_id]['pos'], subConfig):
                    self.graph.add_node(label, pos=point, color='green')
                    self.graph.add_edge(label, node_id)
                    return True
            return False

        if not connect_point("start", checkedStartList[0]) or not connect_point("goal", checkedGoalList[0]):
            return []

        try:
            highLevelPath = nx.shortest_path(self.graph, "start", "goal")
        except nx.NetworkXNoPath:
            return []

        # Build full path from stored sub-paths
        fullPath = [checkedStartList[0]]
        for i in range(len(highLevelPath) - 1):
            u = self.graph.nodes[highLevelPath[i]]['pos']
            v = self.graph.nodes[highLevelPath[i + 1]]['pos']
            edge_key = (tuple(u), tuple(v))
            edge_key_rev = (tuple(v), tuple(u))
            if edge_key in self.edgePaths:
                pathSegment = self.edgePaths[edge_key][1:]
            elif edge_key_rev in self.edgePaths:
                pathSegment = list(reversed(self.edgePaths[edge_key_rev]))[1:]
            else:
                pathSegment = [v]
            fullPath.extend(pathSegment)

        self.solution_path = fullPath  # <- ✅ für Visualisierung
        return fullPath

    def learnSinglePoint(self, pos, node_id, config=None, verbose=False):
        """Try to insert a node into the high-level roadmap based on subplanner visibility."""
        if config is None:
            config = {}

        connected_guards = []
        g_vis = None

        for comp in nx.connected_components(self.graph):
            for g in comp:
                if self.graph.nodes[g].get('nodeType') != 'Guard':
                    continue
                guard_pos = self.graph.nodes[g]['pos']

                path = self.subPlanner.planPath([pos], [guard_pos], config)
                if path and len(path) >= 2:
                    self.edgePaths[(tuple(pos), tuple(guard_pos))] = path
                    if g_vis is None:
                        g_vis = g
                    else:
                        self.graph.add_node(node_id, pos=pos, nodeType='Connection', color='blue')
                        self.graph.add_edge(node_id, g)
                        self.graph.add_edge(node_id, g_vis)
                        if verbose:
                            print(f"✔ Verbindung zu Guards {g_vis} und {g}")
                        return "connection", [g_vis, g]

        if g_vis is None:
            self.graph.add_node(node_id, pos=pos, nodeType='Guard', color='red')
            if verbose:
                print(f"➕ Neuer Guard: {node_id}")
            return "guard", []
        else:
            if verbose:
                print("✖ Sichtbar zu nur einem Guard → Verworfen")
            return "rejected", []
