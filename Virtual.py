import matplotlib.pyplot as plt
import numpy as np
import random
import GraphDomain as gd
import LineHelp as lh
from shapely.geometry import Polygon
HORIZONTAL = 1
VERITCAL = 2

class VirDrone:
    def __init__(self,x,y,world):
        self.x = x
        self.y = y
        self.world = world
        self.yaw_deg = 00
    
        self.northBlocked = False
        self.eastBlocked = False
        self.westBlocked = False
        self.southBlocked = False



class Rectangle:
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
    def print(self):
        print("Starts at {} , {} and is {} long by {} tall".format(self.x,self.y,self.width,self.height))


class Block(Rectangle):
    def __init__(self, x, y, width, height, flyable):
        super().__init__( x, y, width, height)

        self.flyable = flyable
        self.node = gd.Node(x+(width/2),y+(height/2),self)
    def inside(self,x,y):
        return True if (self.x < x and self.x + self.width > x and self.y < y and self.y + self.height > y ) else False
    def fullInside(self,x,y):
        return True if (self.x <= x and self.x + self.width >= x and self.y <= y and self.y + self.height >= y ) else False

    def print(self):
        super().print()
        print("Is partial {}\n".format(self.flyable))

    def getLines(self):
        return [
                [gd.Node(self.x,self.y),gd.Node(self.x,self.y+self.height)],
                [gd.Node(self.x,self.y),gd.Node(self.x+self.width,self.y)],
                [gd.Node(self.x,self.y+self.height),gd.Node(self.x+self.width,self.y+self.height)],
                [gd.Node(self.x+self.width,self.y),gd.Node(self.x+self.width,self.y+self.height)]
               ]


class World(Rectangle):
    def __init__(self, x, y, width, height, color = None , goals = None):
        super().__init__( x, y, width, height)
        # if ( color is not None ):
        #     self.patch = plt.Rectangle((self.x, self.y), self.width, self.height, facecolor=color, edgecolor='#202020')
        self.blocks = [Block(self.x,self.y,self.width,self.height,True)]
        self.drone = VirDrone(x,y,self)
        self.graph = gd.Graph()
        self.obs = []
        self.goals = goals
        self.currentGoal = goals[0] if (goals != None) else None


    def split(self, x, y, dir):
        
        for i in self.blocks:
            if (i.inside(x,y)):
                if dir == VERITCAL:
                    self.blocks.append(Block(x , i.y , ((i.x + i.width)-x) , i.height,True))
                    i.width = x - i.x
                    i.node.x = i.x + (i.width/2)
                if dir == HORIZONTAL:
                    self.blocks.append(Block( i.x , y , i.width , ((i.y + i.height)-y),True))
                    i.height = y - i.y
                    i.node.y = i.y + (i.height/2)
            

    def updateGraph(self):
        #print("updating the graph")
        self.graph.edges = []
        self.graph.nodes = []
        for i in self.blocks:
            for j in self.blocks:
                if(i.x+i.width == j.x   #shares same x value
                and ((j.y >= i.y and j.y <= i.y + i.height) or (i.y >= j.y and i.y <= j.y + j.height)) # 1st y is inbetween 2nd or 2nd is between 1st
                and(j.flyable==True and i.flyable==True)): #both are flyable
                    self.graph.edges.append(gd.Edge(i.node,j.node))

                if(i.y + i.height == j.y 
                and ((j.x >= i.x and j.x <= i.x + i.width) or (i.x >= j.x and i.x <= j.x + j.width))
                and(j.flyable==True and i.flyable==True)):
                    self.graph.edges.append(gd.Edge(i.node,j.node))
            
            self.graph.nodes.append(i.node)#TODO check valid change from i
    

    def combine(self):
        # found = True
        # while(found):
        found = False
        for i in self.blocks:
            for j in self.blocks:
                if(i.y == j.y and i.height == j.height and i.x+i.width == j.x and j.flyable and i.flyable):
                    i.width = i.width + j.width
                    self.blocks.remove(j)
                    found = True
                elif(i.x == j.x and i.width == j.width and i.y+i.height == j.y and j.flyable and i.flyable):
                    i.height = i.height + j.height
                    self.blocks.remove(j)
                    found = True

        self.updateGraph()

    def quadTree(self):
        foundOne = False
        for i in self.blocks:
            for j in self.obs:
                iLines = i.getLines()
                jLines = j.getLines()
                for k in iLines:
                    for l in jLines:
                        if ((lh.intersect(k[0],k[1],l[0],l[1])) and (i.width > 2 or i.height > 2)):
                            foundOne = True
                            break
                    if(foundOne):
                        break
                if(
                    (i.inside(j.x,j.y) or
                    i.inside(j.x,j.y+j.height) or
                    i.inside(j.x+j.width,j.y) or
                    i.inside(j.x+j.width,j.y+j.height)) and (i.width > 2 or i.height > 2)):
                    foundOne = True
            if(foundOne):
                if(i.width >= i.height):
                    self.split(i.node.x, i.node.y,VERITCAL)
                else:
                    self.split(i.node.x, i.node.y,HORIZONTAL)
                
                break
        self.updateGraph()
        return foundOne

    def createObstacles(self, onum, minVal, maxVal):
        self.obs = []
        while( len(self.obs) < onum ):
            x = random.uniform(0.0, self.width)
            y = random.uniform(0.0, self.height)
            w = random.randrange(minVal, maxVal, 1)
            h = random.randrange(minVal, maxVal, 1)
            #w = random.uniform(0.1, owidth)
            #h = random.uniform(0.1, oheight)
            if ( x + w ) > self.width:
                w = self.width - x
            if ( y + h ) > self.height:
                h = self.height - y
            buildObs = Block(x,y, w, h, True)
            found = False
            for o in self.obs:
                obsLines = buildObs.getLines()
                listLines = o.getLines()
                for k in obsLines:
                    for l in listLines:
                        if ((lh.intersect(k[0],k[1],l[0],l[1]))):
                            found = True
                            break
                if(
                    (
                        buildObs.inside(o.x,o.y) or
                        buildObs.inside(o.x,o.y+o.height) or
                        buildObs.inside(o.x+o.width,o.y) or
                        buildObs.inside(o.x+o.width,o.y+o.height) or
                        o.inside(buildObs.x,buildObs.y) or
                        o.inside(buildObs.x,buildObs.y+buildObs.height) or
                        o.inside(buildObs.x+buildObs.width,buildObs.y) or
                        o.inside(buildObs.x+buildObs.width,buildObs.y+buildObs.height)
                    )
                    
                    ):
                    found = True
                    break
            if ( not found ):
                self.obs = self.obs + [buildObs]





    def adjustForSize(self):
        for i in self.blocks:
            i.node.weight = i.node.weight * ((i.width*i.height)/(self.width*self.height))

    def getDroneBlock(self):
        for i in self.blocks:
            return (i.inside(self.drone.x,self.drone.y))
        return None

    def getGoalBlock(self):
        for i in self.blocks:
            return (i.inside(self.currentGoal.x,self.currentGoal.y))
        return None

    def bestNode(self,node):
        bestNode = None
        for i in self.graph.edges:
            if(i.nodeOne is node):
                if((bestNode is None) or i.nodeTwo.weight < bestNode.weight):
                    bestNode = i.nodeTwo    
        return bestNode

    def print(self):
        super().print()
        print("World contains the following blocks:\n")
        for i in self.blocks:
            i.print()
            plt.Rectangle((i.x, i.y), i.width, i.height, edgecolor='#202020')

        print("World has the following graph : \n")
        self.graph.print()
        

