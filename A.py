import heapq

class Node:
    def __init__(self, name, hCost):
        self.name = name
        self.hCost = hCost
        self.gCost = float('inf')
        self.fCost = float('inf')
        self.adjNodes = {}
        self.preNode = None

    def addAdjNode(self, neighbor, weight):
        self.adjNodes[neighbor] = weight

    def getHCost(self):
        return self.hCost

    def getGCost(self):
        return self.gCost

    def getFCost(self):
        return self.fCost

    def getAdjNode(self):
        return self.adjNodes.keys()

    def getWeight(self, neighbor):
        return self.adjNodes[neighbor]

    def refreshPreNode(self, preNode):
        self.preNode = preNode

    def refreshGCost(self, gCost):
        self.gCost = gCost

    def refreshFCost(self):
        self.fCost = self.gCost + self.hCost

    def getPreNode(self):
        return self.preNode

class Graph:
    def __init__(self):
        self.nodeList = {}

    def addNode(self, name, hCost):
        self.nodeList[name] = Node(name, hCost)

    def addEdge(self, fromNode, toNode, weight):
        self.nodeList[fromNode].addAdjNode(toNode, weight)
        self.nodeList[toNode].addAdjNode(fromNode, weight)

    def greedyBestFirstSearch(self, startNode, goalNode):
        openList = []
        heapq.heappush(openList, (self.nodeList[startNode].getHCost(), startNode))
        closedList = set()

        while openList:
            _, currentNode = heapq.heappop(openList)
            closedList.add(currentNode)

            if currentNode == goalNode:
                return self.reconstructPath(goalNode)

            for neighbor in self.nodeList[currentNode].getAdjNode():
                if neighbor in closedList:
                    continue
                if (self.nodeList[neighbor].getHCost(), neighbor) not in openList:
                    self.nodeList[neighbor].refreshPreNode(currentNode)
                    heapq.heappush(openList, (self.nodeList[neighbor].getHCost(), neighbor))

        return None

    def aStarSearch(self, startNode, goalNode):
        openList = []
        self.nodeList[startNode].refreshGCost(0)
        self.nodeList[startNode].refreshFCost()
        heapq.heappush(openList, (self.nodeList[startNode].getFCost(), startNode))
        closedList = set()

        while openList:
            _, currentNode = heapq.heappop(openList)
            closedList.add(currentNode)

            if currentNode == goalNode:
                return self.reconstructPath(goalNode)

            for neighbor in self.nodeList[currentNode].getAdjNode():
                if neighbor in closedList:
                    continue

                tentativeGCost = self.nodeList[currentNode].getGCost() + self.nodeList[currentNode].getWeight(neighbor)
                if tentativeGCost < self.nodeList[neighbor].getGCost():
                    self.nodeList[neighbor].refreshPreNode(currentNode)
                    self.nodeList[neighbor].refreshGCost(tentativeGCost)
                    self.nodeList[neighbor].refreshFCost()
                    if (self.nodeList[neighbor].getFCost(), neighbor) not in openList:
                        heapq.heappush(openList, (self.nodeList[neighbor].getFCost(), neighbor))

        return None

    def reconstructPath(self, goalNode):
        path = []
        currentNode = goalNode
        while currentNode is not None:
            path.append(currentNode)
            currentNode = self.nodeList[currentNode].getPreNode()
        path.reverse()
        return path

# Initialize graph with heuristic costs and edges
dataHCost = {
    "Oradea": 380,
    "Zerind": 374,
    "Arad": 366,
    "Timisoara": 329,
    "Lugoj": 244,
    "Mehadia": 241,
    "Drobeta": 242,
    "Sibiu": 253,
    "Rimnicu Vilcea": 193,
    "Craiova": 160,
    "Pitesti": 100,
    "Fagaras": 176,
    "Bucharest": 0,
    "Giurgiu": 77,
}

dataEdge = {
    "Oradea-Zerind": 71,
    "Oradea-Sibiu": 151,
    "Zerind-Arad": 75,
    "Arad-Sibiu": 140,
    "Sibiu-Fagaras": 99,
    "Fagaras-Bucharest": 211,
    "Arad-Timisoara": 118,
    "Timisoara-Lugoj": 111,
    "Lugoj-Mehadia": 70,
    "Sibiu-Rimnicu Vilcea": 80,
    "Rimnicu Vilcea-Pitesti": 97,
    "Rimnicu Vilcea-Craiova": 146,
    "Craiova-Pitesti": 138,
    "Pitesti-Bucharest": 101,
    "Bucharest-Giurgiu": 90,
    "Mehadia-Drobeta": 75,
    "Drobeta-Craiova": 120,
}

def initGraph(dataHCost, dataEdge):
    g = Graph()
    for node, hCost in dataHCost.items():
        g.addNode(node, hCost)
    for edge, weight in dataEdge.items():
        fromNode, toNode = edge.split('-')
        g.addEdge(fromNode, toNode, weight)
    return g

# Test the Greedy Best-First Search and A* Search
g = initGraph(dataHCost, dataEdge)
print('Greedy Best-First Search Path:', g.greedyBestFirstSearch("Oradea", "Bucharest"))
print('A* Search Path:', g.aStarSearch("Oradea", "Bucharest"))