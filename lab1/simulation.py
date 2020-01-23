#!/usr/bin/python3

# Simulation For Car

import math
#import numpy as np

A = 1000         #X ceiling(mm)
B = 1000         #Y ceiling(mm)
T = .001         #Sampling Period(s)
M = 25           #Magnometer magnitude (+-6)(uT)
R = 90           #Radius of Wheel (mm)
L = 180          #Length of Robot (mm)
theta_1 = 0      #Crit angle 1 (rad)
theta_2 = 0      #Crit angle 2 (rad)
theta_3 = 0      #Crit angle 3 (rad)
theta_4 = 0      #Crit angle 4 (rad)


w_curve = 0      #Angular Velocity for State Dynamics

#STATE
x = 0            #X coord
y = 0            #Y coord
theta = 0        #Heading

#NEXT STATE
x_next = 0       #next X coord
y_next = 0       #next Y coord
theta_next = 0   #next Heading


def pwm_to_velocity(pwm):
    if pwm >= 150: #Max Speed Forward
        w = 6.5
    elif pwm >= 110:#Fast forward
        w = 0.0386*pwm + 0.582
    elif pwm >= 91:#Slow forward
        w = 0.254*pwm - 23.114
    elif pwm >= 86:#Deadzone
        w = 0
    elif pwm >= 70:#Slow Backwards
        w = .297*pwm - 25.456
    elif pwm >= 60:#Fast Backwards
        w = .0756*pwm - 10.038
    else:#Max Speed Reverse
        w = -6.33
    return w*R


def state_dynamics(vl, vr): #Updates State
    global L
    global w_curve
    global theta
    global T
    global x
    global y
    global x_next
    global y_next
    global theta_next
    if vl != vr: #If the robot is following a curved path
        r_curve = L*(vl+vr)/(2*(vr-vl)) #calculate radius of curvature
        w_curve = (vr-vl)*L             #calculate angular velocity
        ccx = x - r_curve*math.sin(theta)   #calculate center of curvature (x coord)
        ccy = y + r_curve*math.cos(theta)   #calculate center of curvature (y coord)
        x_next = (x - ccx)*math.cos(w_curve*T)-(y-ccy)*math.sin(w_curve*T) + ccx    #calc next x coord
        y_next = (x - ccx)*math.sin(w_curve*T)+(y-ccy)*math.cos(w_curve*T) + ccy    #calc next y coord
        theta_next = theta + w_curve*T  #calc next theta
    else:   #If the robot is following a straight path
        x_next = vl*T*math.cos(theta) + x    #calc next x coord
        y_next = vl*T*math.sin(theta) + y    #calc next y coord
        theta_next = theta                   #maintain heading
    return


def state_checker(vl, vr):  #makes sure the next state is valid, corrects it if not.
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


def critical(): #Determines Critical Angles
    global B
    global A
    global x
    global y
    global theta
    global M
    global theta_1
    global theta_2
    global theta_3
    global theta_4
    theta_1 = math.atan2(B-y, A-x)
    theta_2 = math.atan2(B-y, -x) + math.pi
    theta_3 = math.atan2(y,x) + math.pi
    theta_4 = math.atan2(-y, A-x)
    return


def the_d(special_theta):
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
    global theta
    global M


    v_left = pwm_to_velocity(pwm_left)
    v_right = pwm_to_velocity(pwm_right)
    state_dynamics(v_left, v_right)
    state_checker(v_left, v_right)
    critical()
    d1 = the_d(theta)
    theta_x = theta - math.pi/2
    if theta_x < 0:
        theta_x = theta_x + math.pi*2
    elif theta_x > math.pi*2:
        theta_x = theta_x - math.pi*2
    d2 = the_d(theta_x)
    mx = M*math.sin(theta)
    my = M*math.cos(theta)
    gyro = w_curve



if __name__ == "__main__":
    main()
