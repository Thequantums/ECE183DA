import math
import random
import matplotlib.pyplot as plt
import numpy as np

origin = [250, 0, 0]    #Origin point, in form x,y,parent
maxcoords = [500,500]   #Max values of field. x,y form. Assumes bottom left is 0,0
stepsize = 5            #size of step to take (1=> unit vector step)
N = 10000               #Iterations to run
nodesList = [origin]    #list of all nodes
obstacles = [[0,0,100,500],[400,0,500,500],[200,300,400,325],[100,350,250,375]] #obstacle list. Rectangles only, in form xmin, ymin, xmax, ymax
goal = [140,400,150,410]    #goal. Rectangles only, in form xmin, ymin, xmax, ymax


def finddist(node1,node2):  #returns the euclidian distance between two nodes
    dist = math.sqrt(pow(node1[0]-node2[0],2)+pow(node1[1]-node2[1],2))
    return dist


def randomPoint():  #generates a random point
    global maxcoords
    point = [random.randint(0,maxcoords[0]),random.randint(0,maxcoords[1])]
    return point


def obsCheck(point, obstacles): #checks if the point is inside an obstacle. if it is, returns the origin. ##FUTURE## return the closest allowed point
    for o in obstacles:
        if((o[0] < point[0] < o[2]) and (o[1] < point[1] < o[3])):
            return[origin[0],origin[1]]
    return point


def takestep(startnode,targetnode,nodes):   #finds a point one unit step from startnode, in the direction of targetnode. Takes "node" in order to set new node's parent node in node[2]
    dist = finddist(startnode,targetnode)
    newx = ((targetnode[0]-startnode[0]) / dist) * stepsize
    newy = ((targetnode[1] - startnode[1]) / dist) * stepsize
    checkednode = obsCheck([newx + startnode[0], newy +  startnode[1], nodes.index(startnode)], obstacles)
    return checkednode


def findclosest(nodes,newnode): #finds the closest node to newnode in nodelist nodes
    distances = []
    for i in nodes:
        distances.append(finddist(i,newnode))
    return nodes[distances.index(min(distances))]


def checkgoal(nodelist,node,goal):  #checks if the point is within the goal. if not, it sets goalfound to false. if it is, it returns the node path it took and sets goalfound to false
    goalpath = []
    tracenode = []

    if(goal[0] < node[0] < goal[2] and goal[1] < node[1] < goal [3]):
        goalfound = True
        tracenode = node
        goalpath.append(tracenode)
        while(tracenode[2] != 0):
            tracetemp = nodelist[tracenode[2]]
            goalpath.append(tracetemp)
            tracenode = tracetemp
    else:
        goalfound = False
    goalpath.append(origin)
    return [goalfound,goalpath]


def initplot(goal,obstacles):   #initializes plot by drawing obstacles, goal, and origin as well as setting the axes
    goalbox = [[goal[0],goal[2],goal[2],goal[0],goal[0]],[goal[1],goal[1],goal[3],goal[3],goal[1]]]
    for o in obstacles:
        obsbox = [[o[0],o[2],o[2],o[0],o[0]],[o[1],o[1],o[3],o[3],o[1]]]
        plt.plot(obsbox[0], obsbox[1], 'r')
    plt.plot(goalbox[0],goalbox[1],'g')
    plt.xlim(0, maxcoords[0])
    plt.ylim(0, maxcoords[1])


def optimize():
    pass
    #dumb stuff


def main():
    initplot(goal,obstacles)
    for k in range(0,N):
        xrand = randomPoint()
        xnear = findclosest(nodesList,xrand)
        xnew = takestep(xnear,xrand,nodesList)
        [goalbool,goalpath] = checkgoal(nodesList,xnew,goal)
        if (goalbool):
            [xg,yg,zg] = list(zip(*goalpath))
            print('PATH FOUND')
            plt.plot(xg,yg,'y')
            break
        else:
            nodesList.append(xnew)
        print(k)
    x,y = list(zip(*nodesList))
    plt.scatter(x,y,s=1)
    plt.show()

main()