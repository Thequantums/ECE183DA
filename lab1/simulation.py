#!/usr/bin/python3

# Simulation For Car
#####################################################################################
# Input is a file with two columns of data. The first column is the the pwm
# value for the wheel on the left and the second column is the pwm value for
# the wheel on the right. This simulation runs all the pwm values and updates
# the state of the car when processing each pair of values. During each
# iteration it also prints out to a separate file five values that correspond
# to the five measurements the sensors gather on the car: laser distance in
# front of the car, laser distance to the side of the car, magnetometer along
# x and y, and gyroscope reading for rotation along z-axis.
# #####################################################################################
from matplotlib.widgets import Slider

import math  # Needed for all the trigonometry
import numpy as np # Needed for the uncertainty
import matplotlib.pyplot as plt # for plotter

#  Global Constants
A = 530          # X ceiling(mm)
B = 400          # Y ceiling(mm)
T = .1           # Sampling Period(s)
M = 25           # Magnetometer magnitude (+-6)(uT)
R = 40           # Radius of Wheel (mm)
L = 85           # Width of Robot (mm)

# Standard Deviations (square root of variances)
x_var = .1
y_var = .1
theta_var = .01
theta_dot_var = .001

d1_var = 2
d2_var = 2
mx_var = 3
my_var = 3
gyro_var = .01


# Global value for STATE
x = 265            # X coord
y = 200            # Y coord
theta = 0          # Heading
theta_dot = 0      # Angular Velocity

# Global value for NEXT STATE
x_next = 0       # next X coord
y_next = 0       # next Y coord
theta_next = 0   # next Heading

theta_dyn = 0    # Next Angular Velocity

XStateList = []   # state storage for plotting
YStateList = []   # state storage for plotting
ThetaStateList = []   # state storage for plotting

# Initialization for Plotter (Graph size, axes position, etc.
fig = plt.figure(figsize=(8, 8))
plot_ax = plt.axes([0.1, 0.2, 0.8, 0.65])
slider_ax = plt.axes([0.1, 0.05, 0.8, 0.05])
plt.axes(plot_ax)
state_plot, = plt.plot(XStateList, YStateList, 'r')


def pwm_to_velocity(pwm):
    # Function takes pwm value and turns into an angular velocity which is then
    # converted into a translational velocity. Angular velocity is found using
    # experimentation where a non-linear plot is broken down into various linear
    # regions with dead zones and saturation. Experimentation can be found on lab report
    global R
    if pwm >= 150:  # Max Speed Forward, Saturation
        w = 6.5
    elif pwm >= 110:  # Fast forward, Linear Near Saturation
        w = 0.0386*pwm + 0.582
    elif pwm >= 91:  # Slow forward, Linear
        w = 0.254*pwm - 23.114
    elif pwm >= 86:  # Deadzone
        w = 0
    elif pwm >= 70:  # Slow Backwards, Linear
        w = .297*pwm - 25.456
    elif pwm >= 60:  # Fast Backwards, Linear Saturation
        w = .0756*pwm - 10.038
    else:  # Max Speed Reverse, Saturation
        w = -6.33
    return w*R  # Translational Velocity


def state_dynamics(vl, vr):
    # Function updates states based on the velocities of both wheels. This is done using kinematic
    # equations that find the radius of curvature as well as the angular velocity along this
    # curvature. It also includes a condition for when both wheels are at the same velocity since
    # using the equations on this pair of values would result in dividing by zero
    global L
    global theta
    global T
    global x
    global y
    global theta_dyn
    global x_next
    global y_next
    global theta_next
    global x_var
    global y_var
    global theta_var
    global theta_dot_var
    x_dyn = (vl+vr)*math.cos(theta)/2
    y_dyn = (vl+vr)*math.sin(theta)/2
    theta_dyn = (vr-vl)/L
    x_next = x + x_dyn*T + np.random.normal(0,x_var)
    y_next = y + y_dyn*T + np.random.normal(0,y_var)
    theta_next = theta + theta_dyn*T + np.random.normal(0, theta_var)
    if theta_next < 0:  # Correct orientation for below 0 pi and above 2 pi
        theta_next = theta_next + math.pi * 2
    elif theta_next > math.pi * 2:
        theta_next = theta_next - math.pi * 2
    return


def state_checker(vl, vr):
    # This function checks the resulting state update to ensure that the calculated update
    # is a valid state and if it is not then it attempts to call state dynamics with new
    # velocities to properly update the states. These new velocities are given on the assumption
    # that if the next state cannot be reached and we know which axis is incorrect as well as the
    # orientation of our vehicle, then we can predict which wheel is making contact with the wall
    global x
    global y
    global theta
    global x_next
    global y_next
    global theta_next
    if (x_next <= 0 or x_next >= A) and (y_next <= 0 or y_next >= B):  # Both wheels stuck, stay in current state
        x_next = x
        y_next = y
        theta_next = theta
    elif x_next <= 0 or x_next >= A:  # Error along x-axis
        if theta < math.pi/2:
            state_dynamics(vl, 0)  # Pivot along right-wheel
            if (x_next <= 0 or x_next >= A) or (y_next <= 0 or y_next >= B):  # If another wheel gets stuck, stay in current state
                x_next = x
                y_next = y
                theta_next = theta
        elif theta < math.pi:
            state_dynamics(0, vr)  # Pivot along left-wheel
            if (x_next <= 0 or x_next >= A) or (y_next <= 0 or y_next >= B):  # If another wheel gets stuck, stay in current state
                x_next = x
                y_next = y
                theta_next = theta
        elif theta < 3*math.pi/2:
            state_dynamics(vl, 0)  # Pivot along right-wheel
            if (x_next <= 0 or x_next >= A) or (y_next <= 0 or y_next >= B):  # If another wheel gets stuck, stay in current state
                x_next = x
                y_next = y
                theta_next = theta
        else:
            state_dynamics(0, vr)  # Pivot along left-wheel
            if (x_next <= 0 or x_next >= A) or (y_next <= 0 or y_next >= B):  # If another wheel gets stuck, stay in current state
                x_next = x
                y_next = y
                theta_next = theta
    elif y_next <= 0 or y_next >= B:  # Error along x-axis
        if theta < math.pi/2:
            state_dynamics(0, vr)  # Pivot along left-wheel
            if (x_next <= 0 or x_next >= A) or (y_next <= 0 or y_next >= B):  # If another wheel gets stuck, stay in current state
                x_next = x
                y_next = y
                theta_next = theta
        elif theta < math.pi:
            state_dynamics(vl, 0)  # Pivot along right-wheel
            if (x_next <= 0 or x_next >= A) or (y_next <= 0 or y_next >= B):  # If another wheel gets stuck, stay in current state
                x_next = x
                y_next = y
                theta_next = theta
        elif theta < 3*math.pi/2:
            state_dynamics(0, vr)  # Pivot along left-wheel
            if (x_next <= 0 or x_next >= A) or (y_next <= 0 or y_next >= B):  # If another wheel gets stuck, stay in current state
                x_next = x
                y_next = y
                theta_next = theta
        else:
            state_dynamics(vl, 0)  # Pivot along right-wheel
            if (x_next <= 0 or x_next >= A) or (y_next <= 0 or y_next >= B):  # If another wheel gets stuck, stay in current state
                x_next = x
                y_next = y
                theta_next = theta
    # If this ladder was all skipped then the current is legal and no corrections had to be made
    return


def the_d(special_theta):
    # This function calculates the distance measured by a laser given your current state.
    # It works by calculating the distance to all four walls and choosing the minimum
    # positive value from these. We ignore negative values because they're invalid and we
    # want min because it's the first to hit a wall which reflects reality of laser
    global A
    global B
    global x
    global y
    d = [0, 0, 0, 0] # initialize to zero
    if (math.pi / 2 + .1 >= special_theta >= math.pi / 2 - .1)\
        or (3 * math.pi / 2 + .1 >= special_theta >= 3 * math.pi / 2 - .1):  # To handle large numbers
        d[0] = 1000000
        d[2] = 1000000
    else:
        d[0] = (A - x) / math.cos(special_theta)
        d[2] = -x / math.cos(special_theta)
    if (.1 >= special_theta >= 0)\
        or (2*math.pi >= special_theta >= 2*math.pi - .1)\
        or (math.pi + .1 >= special_theta >= math.pi - .1):  # To handle large numbers
        d[1] = 1000000
        d[3] = 1000000
    else:
        d[1] = (B - y) / math.sin(special_theta)
        d[3] = -y / math.sin(special_theta)
    for x in range(4): # To reject negative values, set to mock infinity
        if d[x] < 0:
            d[x] = 1000000

    return min(d)  # chooses min pos


def update(a):              # Function called when slider moves, in order to update plot
    global state_plot
    global XStateList
    global YStateList
    global ThetaStateList
    sana = int(a)
    state_plot.set_xdata(XStateList[0:sana])  # set new x-coordinates of the plotted points
    state_plot.set_ydata(YStateList[0:sana])  # set new y-coordinates of the plotted points
    plt.arrow(XStateList[sana-1],YStateList[sana-1],-3*math.cos(ThetaStateList[sana-1]),-3*math.sin(ThetaStateList[sana-1]),shape = 'full', width = 0.01, head_width = 1)
    fig.canvas.draw_idle()  # redraw the plot


def main():
    # This function reads input from a file and obtains two desired PWM values.
    # It then runs all the other functions to update state and output measurements onto a different file
    global theta
    global M
    global T
    global x
    global y
    global theta
    global theta_dot
    global x_next
    global y_next
    global theta_next
    global theta_dyn
    global theta_dot_var
    global XStateList
    global YStateList
    global ThetaStateList
    global state_plot
    global d1_var
    global d2_var
    global mx_var
    global my_var
    global gyro_var

    i = 0  # Run a counter during each iteration to get timestamp

    myfile = open("pwm_data.txt", 'r')  # file to read
    wr_file = open('simulation_data.txt', 'w')  # file to write observations
    state_file = open('simulation_states', 'w') # file to write states for comparison
    wr_file.write("pwml   pwmr   d1   d2   mx   my   gyro   timestamp\n")
    state_file.write("x y theta theta_dot\n")

    for line in myfile:
        pwml_str = ''
        pwmr_str = ''
        s_begin = 0
        for letter in line:
            if letter != ',' and s_begin == 0:
                pwml_str = pwml_str + letter
            elif letter != ',' and s_begin == 1:
                pwmr_str = pwmr_str + letter
            else:
                s_begin = 1
        pwml = float(pwml_str)
        pwmr = float(pwmr_str)
        v_left = pwm_to_velocity(pwml)  # Get left speed
        v_right = pwm_to_velocity(pwmr)  # Get right speed
        state_dynamics(v_left, v_right)  # Update state
        state_checker(v_left, v_right)  # Check validity of update
        d1 = the_d(theta) + np.random.normal(0, d1_var)  # Distance of forward laser, orientation of laser is same as car
        theta_side = theta - math.pi / 2  # Orientation of second laser is -90 degrees of car's orientation
        # if theta_x < 0:  # Correct orientation for below 0 degrees and above 360 degrees
            # theta_x = theta_x + math.pi * 2
        # elif theta_x >= math.pi * 2:
            # theta_x = theta_x - math.pi * 2
        d2 = the_d(theta_side) + np.random.normal(0, d2_var)  # Distance of side laser
        mx = M * math.sin(theta) + np.random.normal(0, mx_var) # Magnetometer along x-axis
        my = M * math.cos(theta) + np.random.normal(0, my_var) # Magnetometer along y-axis
        gyro = theta_dot + np.random.normal(0, gyro_var)  # Gyro Reading
        timestamp = T * i
        #  Print out values d1,d2,mx,my,gyro, and timestamp
        wr_file.write(str(round(pwml, 2)) + ', ' + str(round(pwmr, 2)) + ', ' + str(round(d1, 2)) + ', '
                      + str(round(d2, 2)) + ', ' + str(round(mx, 2)) + ', ' + str(round(my, 2))
                      + ', ' + str(round(gyro, 2)) + ', ' + str(round(timestamp, 2)) + "\n")


        # Update state

        x = x_next
        y = y_next
        theta = theta_next
        state_file.write(str(round(x, 2)) + ', ' + str(round(y, 2)) + ', ' + str(round(theta, 2)) + ', '
                         + str(round(theta_dot, 2)) + "\n")
        XStateList.append(x)
        YStateList.append(y)

        theta_dot = theta_dyn + np.random.normal(0, theta_dot_var)

        ThetaStateList.append(theta)
        i = i + 1

    myfile.close()
    wr_file.close()
    state_file.close()

    state_plot = plt.plot(XStateList, YStateList, 'r')[0]   #Creates the plot
    a_slider = Slider(slider_ax ,'a' ,0 , len(XStateList), valinit=0, valfmt='%0.0f') #Creates slider with axes, variable, min val, max val, init val, and as integer

    a_slider.on_changed(update) #update the plot when the slider changes
    plt.show()
    print(XStateList)
    print(YStateList)


if __name__ == "__main__":
    main()
