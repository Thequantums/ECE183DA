import math
import random
import sys

import matplotlib.pyplot as plt
import numpy as np
class rrt():


    def __init__(self, origin = [250, 0, 0], maxcoords = [500,500], stepsize = 5, N = 10000, obstacles = [[0, 0, 100, 500], [400, 0, 500, 500], [200, 300, 400, 325],
                     [100, 350, 250, 375]], goal = [140, 400, 150, 410], obstacletype = 'vertex', live = False, divis = 1,scale = 10):
        self.origin = origin  # Origin point, in form x,y,parent
        self.maxcoords = maxcoords  # Max values of field. x,y form. Assumes bottom left is 0,0
        self.stepsize = stepsize  # size of step to take (1=> unit vector step)
        self.N = N  # Iterations to run
        self.obstacles = obstacles  # obstacle list. Rectangles only, in form xmin, ymin, xmax, ymax
        self.goal = goal  # goal. Rectangles only, in form xmin, ymin, xmax, ymax
        self.nodesList = [origin]  # list of all nodes
        self.robotRadius = 5*scale    #Radius of the circular robot estimate
        self.obstacletype = obstacletype    #Whether we're using the vertex obstacle type (manual entry) or the array type (import from obstacleFinder)
        self.live = live    #Whether we're using the "live" plotter or the end-time plotter
        self.divis = divis #draw every divis changes
        self.scale = scale  #scale-up constant for graph
        if self.live:   #Turns on interactive plotting if in live mode
            plt.ion()


    def finddist(self,node1, node2):  # returns the euclidian distance between two nodes
        dist = math.sqrt(pow(node1[0] - node2[0], 2) + pow(node1[1] - node2[1], 2))
        return dist

    def randomPoint(self):  # generates a random point. Uses a larger space than the actual Configuration space in order to increase steps towards the outside of the space
        point = [random.uniform(-100, self.maxcoords[0]+100), random.uniform(-100, self.maxcoords[1]+100)]
        return point


    def obsCheck(self, point, obstacles):  # checks if the point is inside an obstacle. if it is, returns the origin. ##FUTURE## return the closest allowed point
        if self.obstacletype == 'vertex':
            for o in obstacles:
                if (((o[0] < point[0] + self.robotRadius < o[2]) or (o[0] < point[0] - self.robotRadius < o[2])) and (
                        (o[1] < point[1] + self.robotRadius < o[3]) or (o[1] < point[1] - self.robotRadius < o[3]))):
                    return True
        elif self.obstacletype == 'array':  #array type obstacles not currently using robot radius
            xflr = math.floor(point[0]) - 1 #create floor and ceilinged variables to make sure we cover all possible cases.
            yflr = math.floor(point[1]) - 1 #This is because we need to reference the obstacles array, which needs discrete indeces.
            xcl = math.ceil(point[0]) - 1
            ycl = math.ceil(point[1]) - 1
            xmax = obstacles.shape[0]-1
            ymax = obstacles.shape[1]-1

            if xflr >= xmax or xcl >= xmax: #make sure bounds are not violated
                xflr = xcl = xmax
            if yflr >= ymax or ycl >= ymax:
                yflr = ycl = ymax

            if(obstacles[xflr][yflr] or obstacles[xflr][ycl] or obstacles[xcl][yflr] or obstacles[xcl][ycl]): #if the rounded location (via any rounding scheme) is a wall (True in the obstacle array), say so
                return True
        return False


    def pathClear(self, startnode, endnode, obs):   #determines if a path is clear using obsCheck. Does this by canvassing a rectangle with the two input points in opposite corners.
        deadzone = self.stepsize/100    #deadzone because axis perpindicular paths break everything for some reason
        if self.obsCheck(endnode,obs):  #Don't bother if the endpoint is not allowed
            return True
        diffx = (endnode[0] - startnode[0])
        diffy = (endnode[1] - startnode[1])
        if diffx > 0 + deadzone:        #This is in order to use range(), which won't decrement unless it has a negative step
            stepx = 1                   #includes a deadzone to prevent issues with going perpindicular to axis (not sure why this is an issue, but this solves it)
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
                        #Don't allow perfectly perpindicular to axis motion, because of issues mentioned previously
        if stepx == 0:
            return True

        if stepy == 0:
            return True

        for x in range(round(startnode[0]), round((startnode[0] + diffx)), stepx):          #check every node in a rectangle with startnode and endnode as opposite corners to verify path
            for y in range(round(startnode[1]), round((startnode[1] + diffy )), stepy):
                if self.obsCheck([x,y],obs):
                    return True
        return False


    def takestep(self, startnode, targetnode, nodes):  # finds a point one unit step from startnode, in the direction of targetnode. Takes "node" in order to set new node's parent node in node[2]
        dist = self.finddist(startnode, targetnode) #make sure startnode and targetnode are actually different
        if dist != 0:
            newx = ((targetnode[0] - startnode[0]) / dist) * self.stepsize #set new x and y to one unit step, multiplied bu stepsize to expand the step if desired
            newy = ((targetnode[1] - startnode[1]) / dist) * self.stepsize
            newnode = [newx + startnode[0], newy + startnode[1], nodes.index(startnode)]    #sets x and y, as well as storing the parent node
            if self.pathClear(startnode, newnode, self.obstacles):  #Check for obstacles in the path, returns the origin as a dummy node if the point is invalid
                checkednode = self.origin
            else:
                checkednode = newnode
        else:
            checkednode = self.origin
        return checkednode


    def findclosest(self,nodes, newnode):  # finds the closest node to newnode in nodelist nodes
        distances = []
        for i in nodes:
            distances.append(self.finddist(i, newnode))
        return nodes[distances.index(min(distances))]


    def checkgoal(self,nodelist, node, goal):  # checks if the point is within the goal. if not, it sets goalfound to false. if it is, it returns the node path it took and sets goalfound to false
        goalpath = []
        tracenode = []

        if (goal[0] < node[0] < goal[2] and goal[1] < node[1] < goal[3]): #if the node is in the correct range
            goalfound = True        #set goal flag
            tracenode = node        #stores "winning" node in tracenode
            goalpath.append(tracenode)  #adds tracenode to the goal trajectory
            while (tracenode[2] != 0):  #while we havent yet hit the origin
                tracetemp = nodelist[tracenode[2]]  #get the parent node of tracenode
                goalpath.append(tracetemp)      #append parent to trajectory
                tracenode = tracetemp       #set the parent node to the current node and repeat
            goalpath.append(self.origin)  # since the loop breaks once we hit the origin, add the origin to the end
        else:
            goalfound = False

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


    def drawparentlines(self, nodelist):    #draws connecting lines between parent and child nodes
        filterlist = [self.origin]
        for n in nodelist:  #Filters out repeated points for speed of drawing
            if n[2] == 0:
                pass
            else:
                filterlist.append(n)

        for node in filterlist:             #plot each "jump" from child node to parent node
            if(node == nodelist[node[2]]):
                pass
            else:
                jumplist = [node, nodelist[node[2]]]
                plt.plot([jumplist[0][0],jumplist[1][0]],[jumplist[0][1],jumplist[1][1]],'b')
                if self.live and filterlist.index(node) % self.divis == 0:   #Draw these consecutively if in live mode
                    plt.draw()
                    plt.pause(0.0001)


    def optimize(self): #method reserved if we can get RRT* to function
        pass
        # dumb stuff

    def rrt(self, verbose = False, plotting = False):   #Main implementation of RRT
        xg=[]
        yg=[]
        self.initplot(self.goal, self.obstacles)    #initialize plot
        for k in range(0, self.N):      #create (or attempt to create) N nodes
            xrand = self.randomPoint()  #choose a random point
            xnear = self.findclosest(self.nodesList, xrand)     #find the nearest node to the random point
            xnew = self.takestep(xnear, xrand, self.nodesList)  #take one step towards the random point from the nearest node and create a new node
            [goalbool, goalpath] = self.checkgoal(self.nodesList, xnew, self.goal)  #check the new node to see if it's in the goal zone
            if (goalbool):
                [xg, yg, zg] = list(zip(*goalpath))     #If it does succeed, stop creating new nodes and return the goal path
                if verbose == True: #debug info, only dump if verbose
                    print('PATH FOUND')
                break
            else:   #otherwise, just add it to the list and continue
                self.nodesList.append(xnew)
            if verbose == True: #debug info, only dump if verbose
                print(k)
        if plotting == True:# Plot if that's enabled
            self.drawparentlines(self.nodesList)
            if goalbool:
                plt.plot(xg, yg, 'y')
            if self.live:   #If live, draw the goal to the live graph
                plt.draw()
                plt.pause(0.01)
                plt.ioff()
            plt.show()
            xg = (np.array(xg) / self.scale).tolist()   #scale trajectory down from increased scale
            yg = (np.array(yg) / self.scale).tolist()
            trajectory = []
            for i in range(0,len(xg)):
                trajectory.append([xg[i],yg[i]])

        return trajectory[::-1] #return the trajectory to the goal (reverse it, its in goal -> origin order until this line





