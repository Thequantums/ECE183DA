README For Lab 2

This lab was dedicated to the creation and implementation of an RRT pathing algorithm. Our particular version uses image processing to parse an image of a group of obstacles into a Configuration Space, and then finds a path from a (either defined or arbitrary) origin point to a defined goal point. In our case, we were able to get the RRT implementation working reasonably well, though it occasionally overrotates and has difficulty getting to the correct final heading. Despite this error, the RRT implementation we have created is accurate to the theoretical model of the RRT and accurately reflects the state dynamics of our robot.

The following is a description of the files used:

lab3.py: The main file for this program. Uses implementations of the RRT class and the obstaclefinder class to plot a path through a set of obstacles to a defined goal point

obstaclefinder.py: the obstaclefinder class. Initialized with the path to the image to be used as the obstacle map. Uses the Obsfind() method to produce the Configuration space array based on the obstacle map image, the radius of the spherical robot, and a scale-up factor to allow low resolution images to be used.

RRT.py: The RRT class. Uses the rrt() method to plot and return a trajectory/list of timestamped inputs given a configuration space and a set of parameters. Takes the following parameters:
                                        origin: takes the origin point in the form (x,y,theta,_,_) (underscored values should not be changed)
                                        maxcoords: takes the maximum coordinates of the area (top right corner coordinates) assuming the bottom left corner has an x,y coordinate of 0,0
                                        N: Iterations to run
                                        obstacles = obstacles  # Configuration space, including obstacles. In the form of an array with indeces representing x and y coordinates scaled up by the scale variable, in order to make the system sufficiently continuous. 
                                        goal: the value of the goal state
                                        nodesList: list of all nodes
                                        robotRadius: Radius of the circular robot estimate
                                        obstacletype:    Whether we're using the vertex obstacle type (manual entry) or the array type (import from obstacleFinder)
                                        divis: (int) draws nodes every divis new nodes (ie divis = 10, will draw a new frame every 10 new nodes)
                                        scale: (int) scale-up constant for graph
                                        sweetener: (int) determines how often to push the tree towards the goal state. EG a value of 100 means it sets the random point to the goal every 100 iterations
                                        arb: (True/False)  Turns on arbitrary origin point selection
                                        live: (True/False) Turns on interactive plotting if in live mode

maze.bmp: The image of the obstacle map