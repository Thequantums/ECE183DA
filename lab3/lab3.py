import sys

import matplotlib.pyplot as plt
import RRT
import obstaclefinder
import numpy as np


np.set_printoptions(threshold=sys.maxsize)
scale = 10
img = obstaclefinder.imgToObs()
[image,obsimg] = img.obsfind(scale)

plt.imshow(image.T,interpolation='nearest')
r = RRT.rrt(stepsize = 100,N = 10000,obstacles = obsimg, obstacletype = 'array', maxcoords = image.shape,
            origin = [0,50*scale,0],goal = [195*scale,165*scale,225*scale,190*scale],live = True, divis = 10)

trajectory = r.rrt(verbose = True,plotting=True)

print(trajectory)
