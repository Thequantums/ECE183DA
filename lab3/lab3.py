import sys

import matplotlib.pyplot as plt
import RRT
import obstaclefinder
import numpy as np

scale = 10      #set scale-up to increase obstacle accuracy
img = obstaclefinder.imgToObs() #create img from imported picture
[image,obsimg] = img.obsfind(scale,7) #takes scale up factor and obstacle expansion factor and produces display array and obstacle array

plt.imshow(image.T,interpolation='nearest') #show 2D representation of map

#initialize RRT
r = RRT.rrt(stepsize = 100,N = 10000,obstacles = obsimg, obstacletype = 'array', maxcoords = image.shape,
            origin = [0,50*scale,0,0],goal = [195*scale,165*scale,225*scale,190*scale],live = True, divis = 10,scale = scale)
#Perform RRT
trajectory = r.rrt(verbose = True,plotting=True)
#print trajectory
print(trajectory)
