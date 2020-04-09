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
        self.node = gd.Node(x+(width//2),y+(height//2),self)
    def inside(self,x,y):
        return True if (self.x < x and self.x + self.width > x and self.y < y and self.y + self.height > y ) else False

    def print(self):
        super().print()
        print("Is flyable {}\n".format(self.flyable))

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
        self.goals = goals
        self.currentGoal = goals[0] if (goals != None) else None


    def split(self, x, y, dir, flyOne, flyTwo):
        
        for i in self.blocks:
            if (i.inside(x,y)):
                if dir == VERITCAL:
                    self.blocks.append(Block(x , i.y , ((i.x + i.width)-x) , i.height, flyTwo) )
                    i.width = x - i.x
                    i.node.x = i.x + (i.width//2)
                    i.flyable = flyOne
                if dir == HORIZONTAL:
                    self.blocks.append(Block( i.x , y , i.width , ((i.y + i.height)-y), flyTwo) )
                    i.height = y - i.y
                    i.node.y = i.y + (i.height//2)
                    i.flyable = flyOne
            

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

    def checkPaths(self):
        FoundOne = False
        for i in self.graph.edges:
            for j in self.blocks:
                lines = j.getLines()
                for k in lines:
                    if((lh.intersect(i.nodeOne,i.nodeTwo,k[0],k[1])) and j.flyable == False):
                        #print("Crosses a bad block")
                        FoundOne = True
                        areaOne = i.nodeOne.parent.width*i.nodeOne.parent.height
                        areaTwo = i.nodeTwo.parent.width*i.nodeTwo.parent.height
                        
                        #s = 0 if (i.nodeTwo.x==i.nodeOne.x) else (i.nodeTwo.y-i.nodeOne.y)/(i.nodeTwo.x-i.nodeOne.x)

                        if (areaOne >= areaTwo):
                            if(i.nodeOne.parent.width >= i.nodeOne.parent.height):
                                self.split(i.nodeOne.x, i.nodeOne.y,VERITCAL,True,True)
                            if(i.nodeOne.parent.width < i.nodeOne.parent.height):
                                self.split(i.nodeOne.x, i.nodeOne.y,HORIZONTAL,True,True)

                        if (areaTwo >= areaOne):
                            if(i.nodeTwo.parent.width >= i.nodeTwo.parent.height):
                                self.split(i.nodeTwo.x, i.nodeTwo.y,VERITCAL,True,True)
                            if(i.nodeTwo.parent.width < i.nodeTwo.parent.height):
                                self.split(i.nodeTwo.x, i.nodeTwo.y,HORIZONTAL,True,True)
        return FoundOne
    
    def createWallBlock(self,nodeOne, nodeTwo):
        midX = nodeOne.x + ((nodeTwo.x-nodeOne.x)/2)
        midY = nodeOne.y + ((nodeTwo.y-nodeTwo.y)/2)
        safety = 6

        block = Block(0,0,0,0,False)

        if(abs(nodeOne.x-nodeTwo.x)<1):
            #print("vertical")
            block = Block(max(0,midX-safety),max(0,nodeOne.y-safety),safety*2,nodeTwo.y-nodeOne.y+(2*safety),False)
            objLines = block.getLines()

            for i in self.blocks:
                lines = i.getLines()
                for k in lines:
                    if(i.flyable and (
                        i.inside(block.x,block.y) or
                        i.inside(block.x,block.y+block.height) or 
                        i.inside(block.x,block.node.y) or
                        lh.intersect(objLines[0][0],objLines[0][1],k[0],k[1])
                        )):
                        #print("left")
                        self.split(block.x,i.node.y,VERITCAL,True,True)
            self.updateGraph()

            for i in self.blocks:
                lines = i.getLines()
                for k in lines:
                    if( i.flyable and (
                        i.inside(block.x,block.y) or 
                        i.inside(block.x+block.width,block.y) or 
                        i.inside(block.node.x,block.y) or
                        lh.intersect(objLines[1][0],objLines[1][1],k[0],k[1])
                    )):
                        #print("Bottom")
                        self.split(i.node.x,block.y,HORIZONTAL,True,True)
            self.updateGraph()
            
            for i in self.blocks:
                lines = i.getLines()
                for k in lines:
                    if( i.flyable and (
                        lh.intersect(objLines[2][0],objLines[2][1],k[0],k[1]) or 
                        i.inside(block.x,block.y+block.height) or 
                        i.inside(block.x+block.width,block.y+block.height) or
                        i.inside(block.node.x,block.y+block.height)
                    )):
                        #print("Top")
                        self.split(i.node.x,block.y+block.height,HORIZONTAL,True,True)
            self.updateGraph()
            
            for i in self.blocks:
                lines = i.getLines()            
                for k in lines:
                    if( i.flyable and (
                        lh.intersect(objLines[3][0],objLines[3][1],k[0],k[1]) or 
                        i.inside(block.x+block.width,block.y) or
                        i.inside(block.x+block.width,block.y+block.height) or
                        i.inside(block.x+block.width,block.node.y)
                    )):
                        #print("right")
                        self.split(block.x+block.width,i.node.y,VERITCAL,True,True)
            self.updateGraph()

            for i in self.blocks:
                if(i.inside(block.node.x,block.node.y)):
                    i.flyable = False

        else:
            #print("horizontal")
            block = Block(max(0,nodeOne.x-safety),max(0,midY-safety),nodeTwo.x-nodeOne.x+(2*safety),2*safety,False)
            objLines = block.getLines()

        #horizontal

            for i in self.blocks:
                #print("hold")
                lines = i.getLines()
                for k in lines:
                    if( i.flyable and (
                        i.inside(block.x,block.y) or 
                        i.inside(block.x+block.width,block.y) or 
                        i.inside(block.node.x,block.y) or
                        lh.intersect(objLines[1][0],objLines[1][1],k[0],k[1])
                    )):
                        #print("Bottom")
                        self.split(i.node.x,block.y,HORIZONTAL,True,True)
            self.updateGraph()
            
            for i in self.blocks:
                lines = i.getLines()
                for k in lines:
                    if( i.flyable and (
                        i.inside(block.x,block.y) or
                        i.inside(block.x,block.y+block.height) or 
                        i.inside(block.x,block.node.y) or
                        lh.intersect(objLines[0][0],objLines[0][1],k[0],k[1])
                    )):
                        #print("left")
                        self.split(block.x,i.node.y,VERITCAL,True,True)
            self.updateGraph()

            
            for i in self.blocks:
                lines = i.getLines()            
                for k in lines:
                    if( i.flyable and (
                        lh.intersect(objLines[3][0],objLines[3][1],k[0],k[1]) or 
                        i.inside(block.x+block.width,block.y) or
                        i.inside(block.x+block.width,block.y+block.height) or
                        i.inside(block.x+block.width,block.node.y)
                    )):
                        #print("right")
                        self.split(block.x+block.width,i.node.y,VERITCAL,True,True)
            self.updateGraph()

            for i in self.blocks:
                lines = i.getLines()
                for k in lines:
                    if( i.flyable and (
                        #lh.intersect(objLines[2][0],objLines[2][1],k[0],k[1]) or 
                        i.inside(block.x,block.y+block.height) or 
                        i.inside(block.x+block.width,block.y+block.height) or
                        i.inside(block.node.x,block.y+block.height)
                    )):
                        #print("Top")
                        self.split(i.node.x,block.y+block.height,HORIZONTAL,True,True)
            self.updateGraph()

            for i in self.blocks:
                if(i.inside(block.node.x,block.node.y)):
                    i.flyable = False

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
        

