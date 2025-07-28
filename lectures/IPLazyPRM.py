# coding: utf-8
"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein). It is based on the slides given during the course, so please **read the information in theses slides first**

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

from .IPPRMBase import PRMBase
from scipy.spatial import cKDTree
import networkx as nx
import random
import numpy as np

from .IPPerfMonitor import IPPerfMonitor

class LazyPRM(PRMBase):

    def __init__(self, _collChecker):
        super(LazyPRM, self).__init__(_collChecker)
        
        self.graph = nx.Graph()
        self.lastGeneratedNodeNumber = 0
        self.collidingEdges = []
        self.nonCollidingEdges =[]
        self.samplingBounds = None  # 🆕 wichtig!

    def setSamplingBounds(self, bounds):
        """bounds = ((x_min, x_max), (y_min, y_max))"""
        self.samplingBounds = bounds

    def _getRandomPosition(self):
        if hasattr(self, 'samplingBounds'):
            (x_min, x_max), (y_min, y_max) = self.samplingBounds
        else:
            (x_min, x_max), (y_min, y_max) = (0, 22), (0, 22)  # Default fallback

        for _ in range(100):
            pos = [random.uniform(x_min, x_max), random.uniform(y_min, y_max)]
            if not self._collisionChecker.pointInCollision(pos):
                return pos
        raise RuntimeError("Could not find a collision-free sample in bounds")

    @IPPerfMonitor
    def _buildRoadmap(self, numNodes, kNearest):
        
        # generate #numNodes nodes
        addedNodes = []
        for i in range(numNodes):
            pos = self._getRandomPosition()
            self.graph.add_node(self.lastGeneratedNodeNumber, pos=pos)
            addedNodes.append(self.lastGeneratedNodeNumber)
            self.lastGeneratedNodeNumber += 1


        # for every node in graph find nearest neigbhours
        posList = list(nx.get_node_attributes(self.graph,'pos').values())
        #print posList
        kdTree = cKDTree(posList)
        
        # to see when _buildRoadmap has to be called again
        #print addedNodes
        for node in addedNodes:
        #for node in self.graph.nodes():
        # Find set of candidates to connect to sorted by distance
            result = kdTree.query(self.graph.nodes[node]['pos'],k=kNearest)
            for data in result[1]:
                c_node = [x for x, y in self.graph.nodes(data=True)
                if np.array_equal(np.asarray(y['pos']), np.asarray(posList[data]))][0]

                if node!=c_node:
                    if (node, c_node) not in self.collidingEdges:
                        self.graph.add_edge(node,c_node)
                    else:
                        continue
                        #print "not adding already checked colliding edge"
    
    @IPPerfMonitor
    def _checkForCollisionAndUpdate(self,path):
        # first check all nodes
        for nodeNumber in path:
            if self._collisionChecker.pointInCollision(self.graph.nodes[nodeNumber]['pos']):
                self.graph.remove_node(nodeNumber)
                #print "Colliding Node"
                return True
        
        # check all path segments
        for elem in zip(path,path[1:]):
            #print elem
            x = elem[0]
            y = elem[1]
            if self._collisionChecker.lineInCollision(self.graph.nodes()[x]['pos'],self.graph.nodes()[y]['pos']):
                self.graph.remove_edge(x,y)
                self.collidingEdges.append((x,y))
                return True
            else:
                self.nonCollidingEdges.append((x,y))
                                                                                          
            
        return False

    @IPPerfMonitor
    def planPath(self, startList, goalList, config):
        """
        Args:
            startList (array): start position in planning space
            goalList (array) : goal position in planning space
            config (dict): dictionary with roadmap configuration

        Optional:
            config["samplingBounds"]: ((x_min, x_max), (y_min, y_max))
        """
        # 0. reset
        self.graph.clear()
        self.lastGeneratedNodeNumber = 0
        self.collidingEdges = []

        if "samplingBounds" in config:
            self.setSamplingBounds(config["samplingBounds"])
        elif self.samplingBounds is None:
            # Falls nichts gesetzt wurde, nimm globale Limits (z. B. 0–22)
            self.setSamplingBounds(((0, 22), (0, 22)))  # oder beliebig andere Default-Grenzen

        # 1. check start and goal
        checkedStartList, checkedGoalList = self._checkStartGoal(startList, goalList)

        # 2. add start and goal to graph
        self.graph.add_node("start", pos=checkedStartList[0])
        self.graph.add_node("goal", pos=checkedGoalList[0])

        # 3. build initial roadmap
        self._buildRoadmap(config["initialRoadmapSize"], config["kNearest"])

        maxTry = 0
        print("📦 Subplanner-SamplingBounds:", self.samplingBounds)
        while maxTry < config["maxIterations"]:
            try:
                path = nx.shortest_path(self.graph, "start", "goal")
            except:
                self._buildRoadmap(config["updateRoadmapSize"], config["kNearest"])
                maxTry += 1
                continue

            if self._checkForCollisionAndUpdate(path):
                continue
            else:
                return path

        return []


    