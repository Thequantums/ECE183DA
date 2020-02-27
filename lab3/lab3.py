import RRT
import obstaclefinder



img = obstaclefinder.imgToObs()
image = img.obsfind()
r = RRT.rrt(stepsize = 1,N = 20000,obstacles = image, obstacletype = 'array', maxcoords = image.shape,origin = [0,50,0],goal = [195,165,225,190]) #Step of 1 seems to be the max for now. This affects performance, so we want path tracing asap

r.rrt(verbose=True,plotting=True)
