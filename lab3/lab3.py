import RRT
import obstaclefinder


img = obstaclefinder.imgToObs()
image = img.obsfind()
r = RRT.rrt(stepsize = 2,N = 10000,obstacles = image, obstacletype = 'array', maxcoords = image.shape,origin = [0,50,0],goal = [195,165,225,190])

r.rrt(verbose=True,plotting=True)
