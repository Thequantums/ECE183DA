##anku255
from tkinter import Image

import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy import misc

class imgToObs():


    def __init__(self,imagepath = "/Users/bobbe/PycharmProjects/ECE183DA/lab3/testmaze.png"):
        self.image = cv2.imread(imagepath)

    def showimage(self):
        plt.imshow( self.image)
        plt.show()
        cv2.waitKey(0)
        cv2.destroyAllWindows()


    def obsfind(self,scale):
        #Grayscale the image and convert to a boolean array (true for obstacle, false for free space)
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        imagearray = np.transpose(np.array(gray))
        imagearray = imagearray < 255

        templine = []
        temparray = []
        expand = 7
        linei = expand
        pixi = expand
        obsexparray = imagearray.copy()
        # Scale Obstacles to account for robot radius
        for line in imagearray[expand:-expand]:
            for pixel in line[expand:-expand]:
                if pixel == True:
                    for miniline in range(linei - expand, linei + expand):
                        for minipix in range(pixi - expand, pixi + expand):
                            obsexparray[miniline][minipix] = True
                            # print(minipix,',',miniline)
                pixi = pixi + 1
            templine = []
            pixi = expand
            linei = linei + 1

        #Scale up array via duplication
        biggerimageOarray = []
        biggerimgOarrayline = []
        for line in obsexparray.tolist():
            for p in line:
                for i in range(0,scale):
                    biggerimgOarrayline.append(p)
            for i in range(0, scale):
                biggerimageOarray.append(biggerimgOarrayline)
            biggerimgOarrayline = []
        #return Larger true/false array
        obarray = np.array(biggerimageOarray)

        #Scale up array via duplication
        biggerimagearray = []
        biggerimgarrayline = []
        for line in imagearray.tolist():
            for p in line:
                for i in range(0,scale):
                    biggerimgarrayline.append(p)
            for i in range(0, scale):
                biggerimagearray.append(biggerimgarrayline)
            biggerimgarrayline = []
        #return Larger true/false array
        array = np.array(biggerimagearray)


        print("ARRAY PROCESSING DONE")
        return [array,obarray]


