##anku255

import numpy as np
import cv2
import matplotlib.pyplot as plt
class imgToObs():


    def __init__(self,imagepath = "/Users/bobbe/PycharmProjects/ECE183DA/lab3/testmaze.png"):
        self.image = cv2.imread(imagepath)
        plt.imshow(self.image)

    def showimage(self):
        cv2.imshow('Original', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


    def obsfind(self):

        sigma = 0.33
        v = np.median(self.image)
        low = int(max(0, (1.0 - sigma) * v))
        high = int(min(255, (1.0 + sigma) * v))

        edged = cv2.Canny(self.image, low, high)
        imagearray = np.transpose(np.array(edged))
        imagearray = imagearray != 0
        print(imagearray)
        return imagearray


