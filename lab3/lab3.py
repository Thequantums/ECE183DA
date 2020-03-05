import sys

import matplotlib.pyplot as plt
import RRT
import obstaclefinder
import numpy as np

scale = 5      #set scale-up to increase obstacle accuracy
img = obstaclefinder.imgToObs() #create img from imported picture
[image,obsimg] = img.obsfind(scale,10) #takes scale up factor and obstacle expansion factor and produces display array and obstacle array

plt.imshow(image.T,interpolation='nearest') #show 2D representation of map

#initialize RRT
r = RRT.rrt(N = 2000,obstacles = obsimg, obstacletype = 'array', maxcoords = image.shape,
            origin = [35*scale,215*scale,0,0,''],goal = [125*scale,30*scale,0],live = True, divis = 5,scale = scale,arb = False)
#Perform RRT
trajectory = r.rrt(verbose = False,plotting=True)
#print trajectory
print(trajectory)
#print input list
for x in r:
            print(x[4])
#print(r.outputList)

