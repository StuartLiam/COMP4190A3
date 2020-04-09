import Virtual as vr
import GraphDomain as gd
import LineHelp as lh
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
import matplotlib.patches as patches
import numpy as np
import math
HORIZONTAL = 1
VERITCAL = 2

print("starting tests\n")


# def path(motioncommander, drone,node):
#     d = math.sqrt(((drone.x-node.x)**2)+((drone.y-node.y)**2))

#     angle = math.atan(node.y - drone.y) if (node.x-drone.x == 0) else math.atan((node.y-drone.y)/(node.x-drone.x))
#     angle = math.degrees(angle)
#     angle = angle + 90 if (node.x < drone.x and node.y > drone.y) else angle
#     angle = angle + 180 if (node.x < drone.x and node.y < drone.y) else angle
#     angle = angle + 270 if (node.x >drone.x and node.y < drone.y) else angle
#     return [d,angle]

# path = path(0,gd.Node(0,0),gd.Node(10,10))

# print("length {} at angle {}".format(path[0],path[1]))
print ("Creating World\n")
world = vr.World(0,0,100,100)
world.drone.x = 1
world.drone.y = 1
world.print()

#print("adding in a line at 5,5  10,5")
#world.createWallBlock(gd.Node(50,150),gd.Node(150,150))

# print("updating the graph for the split\n")
# world.updateGraph()

# print("testing for bad paths\n")
# while(world.checkPaths()):
#     world.updateGraph()

# print("adding in a line at 20,20  20,10")
#world.createWallBlock(gd.Node(150,50),gd.Node(150,200))

#world.createWallBlock(gd.Node(50,50),gd.Node(150,50))

#

# print("testing for bad paths\n")
# while(world.checkPaths()):
#     world.updateGraph()

world.createObstacles(5,10,50)
#world.obs.append(vr.Block(200,150,50,100,False))
# for i in range():
#     if(not world.quadTree()):
#         break
while(1):
    if(not world.quadTree()):
        break
# world.quadTree()
# world.quadTree()

# print("printing results\n")
# world.print()


# done = False
# while(not done):
#     world.combine()
#     done = True
#     for i in world.blocks:
#         if(((i.width*i.height)/(world.width*world.height))<0.01 and i.partial):
#             done = False
# while(world.checkPaths()):
#     world.updateGraph()

#world.graph.updateWeights(world.blocks[0].node)
# world.adjustForSize()
# print("testing for bad paths\n")
# while(world.checkPaths()):
#     world.updateGraph()
# print("final results\n")

#world.print()




#lh.intersect(gd.Node(1,1),gd.Node(5,1),gd.Node(3,1),gd.Node(5,5))


fig2 = plt.figure()
ax2 = fig2.add_subplot(111, aspect='equal')
ax2.margins(x=100,y=100)
for i in world.obs:
    ax2.add_patch(
        patches.Rectangle(
            (i.x, i.y),
            i.width,
            i.height,
            fill=True      # remove background
        ) )
for i in world.blocks:
    ax2.add_patch(
        patches.Rectangle(
            (i.x, i.y),
            i.width,
            i.height,
            fill=False     # remove background
        ) )


fig2.savefig('rect2.png', dpi=90, bbox_inches='tight')

plt.show()


