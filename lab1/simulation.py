#!/usr/bin/python3

# Simulation For Car
#Fuck this shit
import math
#import numpy as np

A = 1000
A_bot = 10
A_top = 990
B = 1000
B_bot = 1000
T = .001
M = 25
R = 90
L = 180
theta_1 = 0
theta_2 = 0
theta_3 = 0
theta_4 = 0


w_curve = 0

x = 0
y = 0
theta = 0

x_next = 0
y_next = 0
theta_next = 0

#def get_d1(x, y, delta, c_delta, A, B):
 #   if delta < c_delta[0]:
  #      d1 = (A - X) / np.cos(delta)
   # elif delta < c_delta[1]:
    #    if delta < 90:
     #       d1 = (B - Y) / np.sin(delta)
      #  else:
      #      d1 = (B - Y) / np.cos(delta - 90)
    #elif delta < c_delta[3]:
      #  if delta < 180:


#def get_d2():

def pwm_to_velocity(pwm):
    if pwm >= 150:
        w = 6.5
    elif pwm >= 110:
        w = 0.0386*pwm + 0.582
    elif pwm >= 91:
        w = 0.254*pwm - 23.114
    elif pwm >= 86:
        w = 0
    elif pwm >= 70:
        w = .297*pwm - 25.456
    elif pwm >= 60:
        w = .0756*pwm - 10.038
    else:
        w = -6.33
    return w*R

def state_dynamics(vl, vr):
    global L
    global w_curve
    global theta
    global T
    global x
    global y
    global x_next
    global y_next
    global theta_next
    if vl != vr:
        r_curve = L*(vl+vr)/(2*(vr-vl))
        w_curve = (vr-vl)*L
        ccx = x - r_curve*math.sin(theta)
        ccy = y + r_curve*math.cos(theta)
        x_next = (x - ccx)*math.cos(w_curve*T)-(y-ccy)*math.sin(w_curve*T) + ccx
        y_next = (x - ccx)*math.sin(w_curve*T)+(y-ccy)*math.cos(w_curve*T) + ccy
        theta_next = theta + w_curve*T
    else:
        x_next = vl*T*math.cos(theta) + x
        y_next = vl*T*math.sin(theta) + y
        theta_next = theta
    return

def state_checker(vl, vr):
    global x
    global y
    global theta
    global x_next
    global y_next
    global theta_next
    if (x_next < 0 or x_next > A) and (y_next < 0 or y_next > B):
        x_next = x
        y_next = y
        theta_next = theta
    elif x_next < 0 or x_next > A:
        if theta < math.pi/2:
            state_dynamics(vl,0)
            if y_next < 0 or y_next > B:
                x_next = x
                y_next = y
                theta_next = theta
        elif theta < math.pi:
            state_dynamics(0, vr)
            if y_next < 0 or y_next > B:
                x_next = x
                y_next = y
                theta_next = theta
        elif theta < 3*math.pi/2:
            state_dynamics(vl, 0)
            if y_next < 0 or y_next > B:
                x_next = x
                y_next = y
                theta_next = theta
        else:
            state_dynamics(0, vr)
            if y_next < 0 or y_next > B:
                x_next = x
                y_next = y
                theta_next = theta
    elif y_next < 0 or y_next > B:
        if theta < math.pi/2:
            state_dynamics(0,vr)
            if x_next < 0 or x_next > A:
                x_next = x
                y_next = y
                theta_next = theta
        elif theta < math.pi:
            state_dynamics(vl, 0)
            if x_next < 0 or x_next > A:
                x_next = x
                y_next = y
                theta_next = theta
        elif theta < 3*math.pi/2:
            state_dynamics(0, vr)
            if x_next < 0 or x_next > A:
                x_next = x
                y_next = y
                theta_next = theta
        else:
            state_dynamics(vl, 0)
            if x_next < 0 or x_next > A:
                x_next = x
                y_next = y
                theta_next = theta
    return

def critical():
    global B
    global A
    global x
    global y
    global theta
    global M
    global theta_crit_1
    global theta_crit_2
    global theta_crit_3
    global theta_crit_4
    theta_crit_1 = math.atan2(B-y, A-x)
    theta_crit_2 = math.atan2(B-y, -x) + math.pi
    theta_crit_3 = math.atan2(y,x) + math.pi
    theta_crit_4 = math.atan2(-y, A-x)
    return

def outputs(special_theta):
    global M
    global A
    global B
    global theta
    if special_theta < theta_1:
        d = (A-x)/math.cos(special_theta)
    elif special_theta < theta_2:
        if special_theta <= math.pi/2:
            d = (B - y)/math.sin(special_theta)
        else:
            d = (B - y)/math.sin(special_theta - math.pi/2)
    elif special_theta < theta_3:
        if special_theta <= math.pi:
            d = x/ math.cos(math.pi - special_theta)
        else:
            d = x / math.cos(special_theta - math.pi)
    elif special_theta < theta_4:
        if special_theta <= 3*math.pi/2:
            d = y / math.sin(special_theta - math.pi)
        else:
            d = y /math.cos(special_theta - 3*math.pi/2)
    else:
        d = (A-x)/math.cos(2*math.pi-special_theta)
    return d
    
def main():



if __name__ == "__main__":
    main()
