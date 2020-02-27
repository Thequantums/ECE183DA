import math
import random
import sys

import matplotlib.pyplot as plt
import numpy as np
class rrt():


    def __init__(self, origin = [250, 0, 0], maxcoords = [500,500], stepsize = 5, N = 10000, obstacles = [[0, 0, 100, 500], [400, 0, 500, 500], [200, 300, 400, 325],
                     [100, 350, 250, 375]], goal = [140, 400, 150, 410], obstacletype = 'vertex'):
        self.origin = origin  # Origin point, in form x,y,parent
        self.maxcoords = maxcoords  # Max values of field. x,y form. Assumes bottom left is 0,0
        self.stepsize = stepsize  # size of step to take (1=> unit vector step)
        self.N = N  # Iterations to run
        self.obstacles = obstacles  # obstacle list. Rectangles only, in form xmin, ymin, xmax, ymax
        self.goal = goal  # goal. Rectangles only, in form xmin, ymin, xmax, ymax
        self.nodesList = [origin]  # list of all nodes
        self.robotRadius = 5    #Radius of the circular robot estimate
        self.obstacletype = obstacletype


    def finddist(self,node1, node2):  # returns the euclidian distance between two nodes
        dist = math.sqrt(pow(node1[0] - node2[0], 2) + pow(node1[1] - node2[1], 2))
        return dist

    def randomPoint(self):  # generates a random point
        global maxcoords
        point = [random.uniform(0, self.maxcoords[0]), random.uniform(0, self.maxcoords[1])]
        return point


    def obsCheck(self, point, obstacles):  # checks if the point is inside an obstacle. if it is, returns the origin. ##FUTURE## return the closest allowed point
        if self.obstacletype == 'vertex':
            for o in obstacles:
                if (((o[0] < point[0] + self.robotRadius < o[2]) or (o[0] < point[0] - self.robotRadius < o[2])) and (
                        (o[1] < point[1] + self.robotRadius < o[3]) or (o[1] < point[1] - self.robotRadius < o[3]))):
                    return True
        elif self.obstacletype == 'array':
            xflr = math.floor(point[0]) - 1
            yflr = math.floor(point[1]) - 1
            xcl = math.ceil(point[0]) - 1
            ycl = math.ceil(point[1]) - 1
            xmax = obstacles.shape[0]-1
            ymax = obstacles.shape[1]-1

            if xflr >= xmax or xcl >= xmax:
                xflr = xcl = xmax
            if yflr >= ymax or ycl >= ymax:
                yflr = ycl = ymax

            if(obstacles[xflr][yflr] or obstacles[xflr][ycl] or obstacles[xcl][yflr] or obstacles[xcl][ycl]):
                return True
        return False


    def pathClear(self, startnode, endnode, obs):
        deadzone = 10
        if self.obsCheck(endnode,obs):
            return True
        diffx = (endnode[0] - startnode[0])
        diffy = (endnode[1] - startnode[1])
        if diffx > 0 + deadzone:
            stepx = 1
        elif diffx < 0 - deadzone:
            stepx = -1
        else:
            stepx = 0

        if diffy > 0 + deadzone:
            stepy = 1
        elif diffy < 0 - deadzone:
            stepy = -1
        else:
            stepy = 0


        ##############################################################
        #TEMP SCALE SOLUTION
        #################################################################
        scale = 1
        if stepx == 0:
            return True

        if stepy == 0:
            return True

        for x in range(round(startnode[0]*scale),round((startnode[0] + diffx)*scale), stepx):
            for y in range(round(startnode[1]*scale),round((startnode[1] + diffy)*scale), stepy):
                #print(y)
                if self.obsCheck([x/scale,y/scale],obs):
                    return True
        return False


    def takestep(self, startnode, targetnode, nodes):  # finds a point one unit step from startnode, in the direction of targetnode. Takes "node" in order to set new node's parent node in node[2]
        dist = self.finddist(startnode, targetnode)
        if dist != 0:
            newx = ((targetnode[0] - startnode[0]) / dist) * self.stepsize
            newy = ((targetnode[1] - startnode[1]) / dist) * self.stepsize
            newnode = [newx + startnode[0], newy + startnode[1], nodes.index(startnode)]
            if self.pathClear(startnode, newnode, self.obstacles):
                checkednode = startnode
            else:
                checkednode = newnode
        else:
            checkednode = startnode
        return checkednode


    def findclosest(self,nodes, newnode):  # finds the closest node to newnode in nodelist nodes
        distances = []
        for i in nodes:
            distances.append(self.finddist(i, newnode))
        return nodes[distances.index(min(distances))]


    def checkgoal(self,nodelist, node, goal):  # checks if the point is within the goal. if not, it sets goalfound to false. if it is, it returns the node path it took and sets goalfound to false
        goalpath = []
        tracenode = []

        if (goal[0] < node[0] < goal[2] and goal[1] < node[1] < goal[3]):
            goalfound = True
            tracenode = node
            goalpath.append(tracenode)
            while (tracenode[2] != 0):
                tracetemp = nodelist[tracenode[2]]
                goalpath.append(tracetemp)
                tracenode = tracetemp
        else:
            goalfound = False
        goalpath.append(self.origin)
        return [goalfound, goalpath]

    def initplot(self, goal, obstacles):  # initializes plot by drawing obstacles, goal, and origin as well as setting the axes
        goalbox = [[goal[0], goal[2], goal[2], goal[0], goal[0]], [goal[1], goal[1], goal[3], goal[3], goal[1]]]
        if self.obstacletype == 'vertex':
            for o in obstacles:
                obsbox = [[o[0], o[2], o[2], o[0], o[0]], [o[1], o[1], o[3], o[3], o[1]]]
                plt.plot(obsbox[0], obsbox[1], 'r')
        plt.plot(goalbox[0], goalbox[1], 'g')
        plt.xlim(0, self.maxcoords[0])
        plt.ylim(0, self.maxcoords[1])


    def drawparentlines(self, nodelist):
        for node in nodelist:
            if(node == nodelist[node[2]]):
                pass
            else:
                jumplist = [node, nodelist[node[2]]]
                plt.plot([jumplist[0][0],jumplist[1][0]],[jumplist[0][1],jumplist[1][1]],'b')



    def optimize(self):
        pass
        # dumb stuff

    def rrt(self, verbose = False, plotting = False):
        self.initplot(self.goal, self.obstacles)
        for k in range(0, self.N):
            xrand = self.randomPoint()
            xnear = self.findclosest(self.nodesList, xrand)
            xnew = self.takestep(xnear, xrand, self.nodesList)
            [goalbool, goalpath] = self.checkgoal(self.nodesList, xnew, self.goal)
            if (goalbool):
                [xg, yg, zg] = list(zip(*goalpath))
                if verbose == True:
                    print('PATH FOUND')
                break
            else:
                self.nodesList.append(xnew)
            if verbose == True:
                print(k)
        print(self.nodesList)
        x, y, z = list(zip(*self.nodesList))
        if plotting == True:
            self.drawparentlines(self.nodesList)
            if goalbool:
                plt.plot(xg, yg, 'y')
            plt.scatter(x, y, s=1)
            plt.show()
        return [xg, yg]





