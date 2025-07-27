# coding: utf-8

"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein).

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

from lectures.IPPRMBase import PRMBase
import networkx as nx
from scipy.spatial import cKDTree
from lectures.IPPerfMonitor import IPPerfMonitor
import numpy as np

class VisibilityStatsHandler():
    
    def __init__(self):
        self.graph = nx.Graph()
        
    def addNodeAtPos(self,nodeNumber,pos):
        self.graph.add_node(nodeNumber, pos=pos, color='yellow')
        return
    
    def addVisTest(self,fr,to):
        self.graph.add_edge(fr, to)
        return
        
class VisPRM(PRMBase):
    """Class implements an simplified version of a visibility PRM"""

    def __init__(self, _collChecker, _statsHandler = None):
        super(VisPRM, self).__init__(_collChecker)
        self.graph = nx.Graph()
        self.statsHandler = VisibilityStatsHandler() # not yet fully customizable (s. parameters of constructors)
                
    def _isVisible(self, pos, guardPos):
        return not self._collisionChecker.lineInCollision(pos, guardPos)

    @IPPerfMonitor
    def _learnRoadmap(self, ntry):

        nodeNumber = 0
        currTry = 0
        while currTry < ntry:
            #print currTry
            # select a random  free position
            q_pos = self._getRandomFreePosition()
            if self.statsHandler:
                self.statsHandler.addNodeAtPos(nodeNumber, q_pos)
           
            g_vis = None
        
            # every connected component represents one guard
            merged = False
            for comp in nx.connected_components(self.graph): # Für jeden Knoten in der Komponente
                found = False
                merged = False
                for g in comp: # für jeden Gard in der Komponente (ein Guard ist ein Knoten der Sichtbarkeit repräsentiert)
                    if self.graph.nodes()[g]['nodeType'] == 'Guard':
                        if self.statsHandler:
                            self.statsHandler.addVisTest(nodeNumber, g)
                        if self._isVisible(q_pos,self.graph.nodes()[g]['pos']):
                            found = True
                            if g_vis == None:
                                g_vis = g
                            else:
                                self.graph.add_node(nodeNumber, pos = q_pos, color='lightblue', nodeType = 'Connection')
                                self.graph.add_edge(nodeNumber, g)
                                self.graph.add_edge(nodeNumber, g_vis)
                                merged = True
                        # break, if node was visible,because visibility from one node of the guard is sufficient...
                        if found == True: break;
                # break, if connection was found. Reason: computed connected components (comp) are not correct any more, 
                # they've changed because of merging
                if merged == True: # how  does it change the behaviour? What has to be done to keep the original behaviour?
                    break;                    

            if (merged==False) and (g_vis == None):
                self.graph.add_node(nodeNumber, pos = q_pos, color='red', nodeType = 'Guard')
                #print "ADDED Guard ", nodeNumber
                currTry = 0
            else:
                currTry += 1

            nodeNumber += 1

    @IPPerfMonitor
    def planPath(self, startList, goalList, config):
        """
        
        Args:
            start (array): start position in planning space
            goal (array) : goal position in planning space
            config (dict): dictionary with the needed information about the configuration options
            
        Example:
        
            config["ntry"] = 40 
        
        """
        # 0. reset
        self.graph.clear()
        
        # 1. check start and goal whether collision free (s. BaseClass)
        checkedStartList, checkedGoalList = self._checkStartGoal(startList,goalList)
        
        # 2. learn Roadmap
        self._learnRoadmap(config["ntry"])

        # 3. find connection of start and goal to roadmap
        # find nearest, collision-free connection between node on graph and start
        # 3. find connection of start and goal to roadmap
        posList = nx.get_node_attributes(self.graph, 'pos')

        # ❗ Sicherstellen, dass überhaupt Knoten vorhanden sind
        if not posList:
            print("❌ Keine Knoten im Graph – Roadmap konnte nicht erzeugt werden.")
            return []

        kdTree = cKDTree(list(posList.values()))

        # --- Start anbinden ---
        start_connected = False
        result = kdTree.query(checkedStartList[0], k=min(5, len(posList)))
        for idx in np.atleast_1d(result[1]):
            if idx >= len(posList):
                continue
            node_id = list(posList.keys())[idx]
            if not self._collisionChecker.lineInCollision(checkedStartList[0], self.graph.nodes[node_id]['pos']):
                self.graph.add_node("start", pos=checkedStartList[0], color='lightgreen')
                self.graph.add_edge("start", node_id)
                start_connected = True
                break

        # --- Ziel anbinden ---
        goal_connected = False
        result = kdTree.query(checkedGoalList[0], k=min(5, len(posList)))
        for idx in np.atleast_1d(result[1]):
            if idx >= len(posList):
                continue
            node_id = list(posList.keys())[idx]
            if not self._collisionChecker.lineInCollision(checkedGoalList[0], self.graph.nodes[node_id]['pos']):
                self.graph.add_node("goal", pos=checkedGoalList[0], color='lightgreen')
                self.graph.add_edge("goal", node_id)
                goal_connected = True
                break

        if not (start_connected and goal_connected):
            print("❌ Start oder Ziel konnte nicht mit der Roadmap verbunden werden.")
            return []

        # --- Kürzesten Pfad suchen ---
        try:
            path = nx.shortest_path(self.graph, "start", "goal")
        except nx.NetworkXNoPath:
            print("❌ Kein Pfad zwischen Start und Ziel.")
            return []

        return path

        
