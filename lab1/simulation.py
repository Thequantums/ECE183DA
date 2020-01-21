#!/usr/bin/python3

/* Simulation For Car */

import math
from numpy as np


def get_d1(x, y, delta, c_delta, A, B):
    if delta < c_delta[0]:
       d1 = (A - X)/np.cos(delta)
    elif delta < c_delta[1]:
       if delta < 90:
          d1 = (B - Y)/np.sin(delta)
       else:
          d1 = (B - Y)/np.cos(delta - 90)
    elif delta < c_delta[3]:
       if delta < 180:
          

   

def get_d2():





def main():


if __name__ == "__main__":
    main()
