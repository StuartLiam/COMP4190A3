import math

class Node:
    def __init__(self,x,y,parent=None,weight=None):
        #print("node created")
        self.x = x
        self.y = y
        self.parent = parent
        self.weight = weight
        

    def print(self):
        print("Node at {},{} with weight {}".format(self.x,self.y,self.weight))

        

class Edge:
    def __init__(self,nodeOne,nodeTwo):
        self.nodeOne = nodeOne
        self.nodeTwo = nodeTwo
        self.length = self.distance()

    def print(self):
        print("edge between 2 following nodes: ")
        self.nodeOne.print()
        self.nodeTwo.print()
        print("\n")

    def distance(self):
        absX = abs(self.nodeOne.x - self.nodeTwo.x)
        absY = abs(self.nodeOne.y - self.nodeTwo.y)

        return math.sqrt(absX ** 2 + absY ** 2)

        
class Graph:
    def __init__(self,nodes = None, edges = None):
        self.nodes = nodes if (nodes != None) else []
        self.edges = edges if (edges != None) else []

    def print(self):
        print("Has nodes: \n")
        for i in self.nodes:
            i.print()
        print("Has edges: \n")
        for i in self.edges:
            i.print()

    
    def updateWeights(self,goalNode):
        for i in self.nodes:
            i.weight = 999999
        for i in self.nodes:
            if (i is goalNode):
                i.weight = 0
                #print("found goal node")

                for j in self.edges:
                    if(i is j.nodeOne):
                        j.nodeTwo.weight = j.length + i.weight
                        self.weightPass(j.nodeTwo)
                    if(i is j.nodeTwo):
                        j.nodeOne.weight = j.length + i.weight
                        self.weightPass(j.nodeOne)
        #for i in self.edges:
            #i.print()

    def weightPass(self, node):
        #print("hold")
        for j in self.edges:
            #j.print()
            newWeight = j.length + node.weight
            if(node is j.nodeOne and (j.nodeTwo.weight > newWeight)):
                j.nodeTwo.weight = newWeight
                self.weightPass(j.nodeTwo)
            if(node is j.nodeTwo and (j.nodeOne.weight>newWeight)):
                j.nodeOne.weight = newWeight
                self.weightPass(j.nodeOne)

    def bestEdge(self,node):
        currBest = None
        for i in edges:
            if(i.nodeOne is node):
                currBest = i.nodeTwo if (currBest is None or i.nodeTwo.weight < currBest.weight) else currBest
            if(i.nodeTwo is node):
                currBest = i.nodeOne if (currBest is None or i.nodeOne.weight < currBest.weight) else currBest