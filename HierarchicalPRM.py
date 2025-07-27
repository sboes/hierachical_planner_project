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
        """Check visibility using subplanner, but without allowing it to sample new nodes."""

        # 🧼 Subplaner zurücksetzen
        self.subPlanner.graph.clear()

        # Nur Start und Ziel einfügen
        self.subPlanner.graph.add_node("start", pos=pos1)
        self.subPlanner.graph.add_node("goal", pos=pos2)

        # Optional: direkter Versuch einer Verbindung
        if hasattr(self.subPlanner, "_collisionChecker"):
            if self.subPlanner._collisionChecker.lineInCollision(pos1, pos2):
                return False

        # Manuell direkte Kante einfügen
        self.subPlanner.graph.add_edge("start", "goal")

        # Pfadplanung ohne neue Knoten (nutze einen Dummy-Planer oder eine manuelle Abfrage)
        try:
            path = nx.shortest_path(self.subPlanner.graph, "start", "goal")
        except nx.NetworkXNoPath:
            return False

        # Nur wenn gültiger Pfad gefunden → sichtbar
        if path and len(path) >= 2:
            self.edgePaths[(tuple(pos1), tuple(pos2))] = [pos1, pos2]
            return True

        return False

    @IPPerfMonitor
    def _learnRoadmap(self, ntry, subConfig):
        nodeNumber = 0
        currTry = 0

        while currTry < ntry:
            q_pos = self._getRandomFreePosition()
            print(f"\n🔎 Prüfe Punkt {nodeNumber} bei {q_pos}")

            visible_guards = []

            for comp in nx.connected_components(self.graph):
                for g in comp:
                    if self.graph.nodes[g]['nodeType'] == 'Guard':
                        if self._isVisible(q_pos, self.graph.nodes[g]['pos'], subConfig):
                            visible_guards.append(g)
                            print(f"   ✅ Sichtverbindung zu Guard {g} gefunden")
                            if len(visible_guards) >= 2:
                                break
                if len(visible_guards) >= 2:
                    break

            if len(visible_guards) >= 2:
                self.graph.add_node(nodeNumber, pos=q_pos, color='lightblue', nodeType='Connection')
                self.graph.add_edge(nodeNumber, visible_guards[0])
                self.graph.add_edge(nodeNumber, visible_guards[1])
                print(
                    f"🔗 Punkt {nodeNumber} als Connection verbunden mit Guards {visible_guards[0]} und {visible_guards[1]}")
                currTry += 1

            elif len(visible_guards) == 0:
                self.graph.add_node(nodeNumber, pos=q_pos, color='red', nodeType='Guard')
                print(f"🛡️ Punkt {nodeNumber} als neuer Guard hinzugefügt (keine Verbindung möglich)")
                currTry = 0

            else:
                print(f"❌ Punkt {nodeNumber} verworfen (nur eine Sichtverbindung zu Guard {visible_guards[0]})")
                # Nichts wird eingefügt
                currTry += 1  # Du kannst auch `currTry = 0` machen, wenn du willst, dass solche Punkte keinen Fortschritt zählen

            nodeNumber += 1

    @IPPerfMonitor
    def planPath(self, startList, goalList, config):
        self.graph.clear()
        self.edgePaths.clear()
        self.solution_path = []

        subConfig = config.get("subPlannerConfig", {})
        ntry = config.get("ntry", 40)

        checkedStartList, checkedGoalList = self._checkStartGoal(startList, goalList)
        self._learnRoadmap(ntry, subConfig)

        posList = nx.get_node_attributes(self.graph, 'pos')
        if not posList:
            return []

        kdTree = cKDTree(list(posList.values()))

        def connect_point(label, point):
            if len(posList) == 0:
                return False

            k = min(5, len(posList))
            distances, indices = kdTree.query(point, k=k)

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

        fullPath = []

        for i in range(len(highLevelPath) - 1):
            u = self.graph.nodes[highLevelPath[i]]['pos']
            v = self.graph.nodes[highLevelPath[i + 1]]['pos']

            edge_key = (tuple(u), tuple(v))
            edge_key_rev = (tuple(v), tuple(u))

            if edge_key in self.edgePaths:
                pathSegment = self.edgePaths[edge_key]
            elif edge_key_rev in self.edgePaths:
                pathSegment = list(reversed(self.edgePaths[edge_key_rev]))
            else:
                pathSegment = [u, v]

            # Nur gültige 2D-Koordinaten sammeln
            for p in pathSegment:
                if isinstance(p, (list, tuple)) and len(p) == 2:
                    fullPath.append(tuple(p))
                else:
                    print(f"⚠️ Ungültiger Punkt in Pfadsegment übersprungen: {p}")

        self.solution_path = fullPath
        return fullPath

    def learnSinglePoint(self, pos, node_id, config=None, verbose=False):
        if config is None:
            config = {}

        g_vis = None

        for comp in nx.connected_components(self.graph):
            for g in comp:
                if self.graph.nodes[g].get('nodeType') != 'Guard':
                    continue
                guard_pos = self.graph.nodes[g]['pos']

                path = self.subPlanner.planPath([pos], [guard_pos], config)
                if isinstance(path, list) and all(isinstance(p, (list, tuple)) and len(p) == 2 for p in path):
                    if len(path) >= 2:
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
