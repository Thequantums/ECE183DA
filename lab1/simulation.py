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

import math  # Needed for all the trigonometry

#  Global Constants
A = 530          # X ceiling(mm)
B = 400          # Y ceiling(mm)
T = .1           # Sampling Period(s)
M = 25           # Magnetometer magnitude (+-6)(uT)
R = 90           # Radius of Wheel (mm)
L = 180          # Length of Robot (mm)

# Global values for critical angles
# NOTE: A critical angle is the angle of a line from the cars position
# to the one of the four corners in the arena. These angles allow us
# to decide which wall to use when calculating distance measurement
# of a laser.
theta_1 = 0      # Critical angle 1 (rad)
theta_2 = 0      # Critical angle 2 (rad)
theta_3 = 0      # Critical angle 3 (rad)
theta_4 = 0      # Critical angle 4 (rad)

# Global value for angular velocity which directly corresponds to gyro reading
w_curve = 0      # Angular Velocity for State Dynamics

# Global value for STATE
x = 0            # X coord
y = 0            # Y coord
theta = 0        # Heading

# Global value for NEXT STATE
x_next = 0       # next X coord
y_next = 0       # next Y coord
theta_next = 0   # next Heading


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
    global w_curve
    global theta
    global T
    global x
    global y
    global x_next
    global y_next
    global theta_next
    if vl != vr:  # If the robot is following a curved path
        r_curve = L*(vl+vr)/(2*(vr-vl))  # calculate radius of curvature
        w_curve = (vr-vl)*L              # calculate angular velocity
        ccx = x - r_curve*math.sin(theta)   # calculate center of curvature (x coord)
        ccy = y + r_curve*math.cos(theta)   # calculate center of curvature (y coord)
        x_next = (x - ccx)*math.cos(w_curve*T)-(y-ccy)*math.sin(w_curve*T) + ccx    # calc next x coord
        y_next = (x - ccx)*math.sin(w_curve*T)+(y-ccy)*math.cos(w_curve*T) + ccy    # calc next y coord
        theta_next = theta + w_curve*T  # calc next theta
    else:   # If the robot is following a straight path
        x_next = vl*T*math.cos(theta) + x    # calc next x coord
        y_next = vl*T*math.sin(theta) + y    # calc next y coord
        theta_next = theta                   # maintain heading
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
    if (x_next < 0 or x_next > A) and (y_next < 0 or y_next > B):  # Both wheels stuck, stay in current state
        x_next = x
        y_next = y
        theta_next = theta
    elif x_next < 0 or x_next > A:  # Error along x-axis
        if theta < math.pi/2:
            state_dynamics(vl, 0)  # Pivot along right-wheel
            if y_next < 0 or y_next > B:  # If both wheels stuck, stay in current state
                x_next = x
                y_next = y
                theta_next = theta
        elif theta < math.pi:
            state_dynamics(0, vr)  # Pivot along left-wheel
            if y_next < 0 or y_next > B:  # If both wheels stuck, stay in current state
                x_next = x
                y_next = y
                theta_next = theta
        elif theta < 3*math.pi/2:
            state_dynamics(vl, 0)  # Pivot along right-wheel
            if y_next < 0 or y_next > B:  # If both wheels stuck, stay in current state
                x_next = x
                y_next = y
                theta_next = theta
        else:
            state_dynamics(0, vr)  # Pivot along left-wheel
            if y_next < 0 or y_next > B:  # If both wheels stuck, stay in current state
                x_next = x
                y_next = y
                theta_next = theta
    elif y_next < 0 or y_next > B:  # Error along x-axis
        if theta < math.pi/2:
            state_dynamics(0, vr)  # Pivot along left-wheel
            if x_next < 0 or x_next > A:  # If both wheels stuck, stay in current state
                x_next = x
                y_next = y
                theta_next = theta
        elif theta < math.pi:
            state_dynamics(vl, 0)  # Pivot along right-wheel
            if x_next < 0 or x_next > A:  # If both wheels stuck, stay in current state
                x_next = x
                y_next = y
                theta_next = theta
        elif theta < 3*math.pi/2:
            state_dynamics(0, vr)  # Pivot along left-wheel
            if x_next < 0 or x_next > A:  # If both wheels stuck, stay in current state
                x_next = x
                y_next = y
                theta_next = theta
        else:
            state_dynamics(vl, 0)  # Pivot along right-wheel
            if x_next < 0 or x_next > A:  # If both wheels stuck, stay in current state
                x_next = x
                y_next = y
                theta_next = theta
    # If this ladder was all skipped then the current is legal and no corrections had to be made
    return


def critical():  # Determines Critical Angles
    # This function calculates the previously mentioned critical angles, this can
    # be easily done since the corners of the arena are constants and we are aware of
    # our current state
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
    theta_1 = math.atan2(B-y, A-x)  # Angle to the top-right corner
    theta_2 = math.atan2(B-y, -x) + math.pi  # Angle to the top-left corner
    theta_3 = math.atan2(y, x) + math.pi # Angle to the bottom-left corner
    theta_4 = math.atan2(-y, A-x) # Angle to the bottom-right corner
    return


def the_d(special_theta):
    # This function calculates the distance measured by a laser given your current state.
    # It works by comparing the orientation of the laser with the critical angles in order
    # to understand which wall the laser is hitting. Using that we can calculate d using trig
    global M
    global A
    global B
    global theta
    if special_theta < theta_1:  # Hitting right wall
        d = (A-x)/math.cos(special_theta)
    elif special_theta < theta_2:  # Hitting top wall
        if special_theta <= math.pi/2:
            d = (B - y)/math.sin(special_theta)
        else:
            d = (B - y)/math.sin(special_theta - math.pi/2)
    elif special_theta < theta_3:  # Hitting left wall
        if special_theta <= math.pi:
            d = x / math.cos(math.pi - special_theta)
        else:
            d = x / math.cos(special_theta - math.pi)
    elif special_theta < theta_4:  # Hitting bottom wall
        if special_theta <= 3*math.pi/2:
            d = y / math.sin(special_theta - math.pi)
        else:
            d = y / math.cos(special_theta - 3*math.pi/2)
    else:  # Hitting right wall again
        d = (A-x)/math.cos(2*math.pi-special_theta)
    return d


def main():
    # This function reads input from a file and obtains two desired PWM values.
    # It then runs all the other functions to update state and output measurements onto a different file
    global theta
    global M
    global T
    global x
    global y
    global theta
    global x_next
    global y_next
    global theta_next

    i = 0  # Run a counter during each iteration to get timestamp
    # Code in between #'s should be in a loop
    ###################################################################################################
    myfile = open("pwm_data.txt",'r') # file to read
    wr_file = open('simulation_data.txt', 'w') # file to write
    wr_file.write("pwml   pwmr   d1   d2   mx   my   gyro   timestamp\n")

    for line in myfile:
       pwml_str ='';
       pwmr_str = '';
       s_begin = 0;
       for letter in line:
         if letter != ',' and s_begin == 0:
            pwml_str = pwml_str + letter
         elif letter != ',' and s_begin ==1:
            pwmr_str = pwmr_str + letter;
         else:
            s_begin = 1
       pwml = (float)(pwml_str)
       pwmr = (float)(pwmr_str)
       #############################3###########3
       v_left = pwm_to_velocity(pwml)  # Get left speed
       v_right = pwm_to_velocity(pwmr)  # Get right speed
       state_dynamics(v_left, v_right)  # Update state
       state_checker(v_left, v_right)  # Check validity of update
       critical()  # Calculate critical values
       d1 = the_d(theta)  # Distance of forward laser, orientation of laser is same as car
       theta_x = theta - math.pi/2  # Orientation of second laser is -90 degrees of car's     orientation
       if theta_x < 0:  # Correct orientation for below 0 degrees and above 360 degrees
         theta_x = theta_x + math.pi*2
       elif theta_x > math.pi*2:
         theta_x = theta_x - math.pi*2
       d2 = the_d(theta_x)  # Distance of side laser
       mx = M*math.sin(theta)  # Magnetometer along x-axis
       my = M*math.cos(theta)  # Magnetometer along y-axis
       gyro = w_curve  # Gyro Reading
       timestamp = T*i
       ##############################################
       #  write out values d1,d2,mx,my,gyro, and timestamp
       #rounded to 2 decimal places values
       wr_file.write(str(round(pwml,2)) + ', ' + str(round(pwmr,2)) + ', ' + str(round(d1,2)) + ', ' + str(round(d2,2)) + ', ' + str(round(mx,2)) + ', ' + str(round(my,2)) + ', ' + str(round(gyro,2)) + ', ' + str(round(timestamp,2)) + "\n")
       
       # Update state
       x = x_next
       y = y_next
       theta = theta_next
       i = i+1

    #####################################################################################################

    myfile.close()
    wr_file.close()    

if __name__ == "__main__":
    main()
