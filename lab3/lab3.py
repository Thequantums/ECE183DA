import sys

import matplotlib.pyplot as plt
import RRT
import obstaclefinder
import numpy as np


np.set_printoptions(threshold=sys.maxsize)
scale = 10
img = obstaclefinder.imgToObs()
image = img.obsfind(scale)

plt.imshow(image.T,interpolation='nearest')
r = RRT.rrt(stepsize = 100,N = 10000,obstacles = image, obstacletype = 'array', maxcoords = image.shape,origin = [0,50*scale,0],goal = [195*scale,165*scale,225*scale,190*scale]) #Step of 1 seems to be the max for now. This affects performance, so we want path tracing asap

r.rrt(verbose=True,plotting=True)
