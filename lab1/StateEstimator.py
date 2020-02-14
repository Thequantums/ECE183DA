import numpy as np
import math
import matplotlib.pyplot as plt
pmwL = []
pmwR = []

d1 = []
d2 = []
mx = []
my = []
gyro = []
timestamp = []
data = []

w_x = .01
w_y = .01
w_theta = .001
w_theta_dot = .000001

v_d1 = 2.25
v_d2 = 2.25
v_gyro = .0001

###################################################
# Outputs inputVal array and outputVal array
###################################################
Rw = 20
L = 85
d = L / 2
T = 0.1
A = 530
B = 400
###################################################
# Kalman variables
###################################################

x_best = np.array([[260, 200, 0, 0]])  # State     (each state of size n) (x,y,theta,thetaDot)
x_best = x_best.T
Pk = np.array([[100, 0, 0, 0], [0, 100, 0, 0], [0, 0, math.pi, 0], [0, 0, 0, 1]])  # Covariance Matrix    (each element of size nxn)
Qk = np.array([[w_x, 0, 0, 0], [0, w_y, 0, 0], [0, 0, w_theta, 0], [0, 0, 0, w_theta_dot]])  # Environmental Error (from experimentation)(size nxn)
Rk = np.array([[v_d1, 0, 0], [0, v_d2, 0], [0, 0, v_gyro]])  # Sensor Noise (from experimentation)(size nxm)
id_mat = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
Fk = np.array([])  # Prediction Matrix (from linearization) (size nxn)
Gk = np.array([])  # input matrix
Hk = np.array([])  # Sensor Model (from linearization)(size mxn)
K = np.array([])  # Kalman Gain  # Sensor Reading (size m)
xPost = np.array([])  # Posteriori State
PPost = np.array([])  # Posteriori Covariance

x_estimates = [x_best]
P_estimates = [Pk]

##########################################################
# PWM->Velocity
##########################################################


def pwm_to_w_velocity(pwm):
    # Function takes pwm value and turns into an angular velocity which is then
    # converted into a translational velocity. Angular velocity is found using
    # experimentation where a non-linear plot is broken down into various linear
    # regions with dead zones and saturation. Experimentation can be found on lab report
    if pwm >= 150:  # Max Speed Forward, Saturation
        w = 6.5
    elif pwm >= 110:  # Fast forward, Linear Near Saturation
        w = 0.0386 * pwm + 0.582
    elif pwm >= 91:  # Slow forward, Linear
        w = 0.254 * pwm - 23.114
    elif pwm >= 86:  # Deadzone
        w = 0
    elif pwm >= 70:  # Slow Backwards, Linear
        w = .297 * pwm - 25.456
    elif pwm >= 60:  # Fast Backwards, Linear Saturation
        w = .0756 * pwm - 10.038
    else:  # Max Speed Reverse, Saturation
        w = -6.33
    return w  # Translational Velocity


def f_update(state, in_values): #
    global Rw
    global d
    global T
    wl = pwm_to_w_velocity(in_values[0])
    wr = pwm_to_w_velocity(in_values[1])
    a = Rw * (wl + wr) / 2
    Ft = np.array([[1, 0, -a * math.sin(state[2][0]) * T, 0],
                   [0, 1, a * math.cos(state[2][0]) * T, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 0]])
    return Ft


def g_update(state):
    global Rw
    global d
    global T
    gt = np.array([[Rw * math.cos(state[2][0]) * T / 2, Rw * math.cos(state[2][0]) * T / 2],
                   [Rw * math.sin(state[2][0]) * T / 2, Rw * math.sin(state[2][0]) * T / 2],
                   [-Rw * T / (2 * d), Rw * T / (2 * d)],
                   [-Rw / (2 * d), Rw / (2 * d)]])
    return gt


def h_update(state):
    global A
    global B
    front_laser = [0, 0, 0, 0]
    side_laser = [0, 0, 0, 0]
    state_side = state[2][0] - math.pi/2
    if state_side < 0:  # Correct orientation for below 0 degrees and above 360 degrees
        state_side = state_side + math.pi * 2
    elif state_side >= math.pi * 2:
        state_side = state_side - math.pi * 2

    if (math.pi / 2 + .1 >= state[2][0] >= math.pi / 2 - .1) \
            or (3 * math.pi / 2 + .1 >= state[2][0] >= 3 * math.pi / 2 - .1):  # To handle large numbers
        front_laser[0] = 1000000
        front_laser[2] = 1000000
    else:
        front_laser[0] = (A - state[0][0]) / math.cos(state[2][0])
        front_laser[2] = -state[0][0] / math.cos(state[2][0])
    if (.1 >= state[2][0] >= 0) \
            or (2 * math.pi >= state[2][0] >= 2 * math.pi - .1) \
            or (math.pi + .1 >= state[2][0] >= math.pi - .1):  # To handle large numbers
        front_laser[1] = 1000000
        front_laser[3] = 1000000
    else:
        front_laser[1] = (B - state[1][0]) / math.sin(state[2][0])
        front_laser[3] = - state[1][0] / math.sin(state[2][0])
    for x in range(4):  # To reject negative values, set to mock infinity
        if front_laser[x] < 0:
            front_laser[x] = 1000000

    if (math.pi / 2 + .1 >= state_side >= math.pi / 2 - .1) \
            or (3 * math.pi / 2 + .1 >= state_side >= 3 * math.pi / 2 - .1):  # To handle large numbers
        side_laser[0] = 1000000
        side_laser[2] = 1000000
    else:
        side_laser[0] = (A - state[0][0]) / math.cos(state_side)
        side_laser[2] = -state[0][0] / math.cos(state_side)
    if (.1 >= state_side >= 0) \
            or (2 * math.pi >= state_side >= 2 * math.pi - .1) \
            or (math.pi + .1 >= state_side >= math.pi - .1):  # To handle large numbers
        side_laser[1] = 1000000
        side_laser[3] = 1000000
    else:
        side_laser[1] = (B - state[1][0]) / math.sin(state_side)
        side_laser[3] = - state[1][0] / math.sin(state_side)
    for x in range(4):  # To reject negative values, set to mock infinity
        if side_laser[x] < 0:
            side_laser[x] = 1000000

    sel_front_laser = front_laser.index(min(front_laser))
    sel_side_laser = side_laser.index(min(side_laser))

    if sel_front_laser == 0:
        MAA = -1 / math.cos(state[2][0])
        MBA = 0
        MCA = (A - state[0][0]) * (math.sin(state[2][0]) / pow(math.cos(state[2][0]), 2.0))
    elif sel_front_laser == 1:
        MAA = 0
        MBA = -1 / math.sin(state[2][0])
        MCA = -(B - state[1][0]) * (math.cos(state[2][0]) / pow(math.sin(state[2][0]), 2.0))
    elif sel_front_laser == 2:
        MAA = -1 / math.cos(state[2][0])
        MBA = 0
        MCA = -state[0][0] * (math.sin(state[2][0]) / pow(math.cos(state[2][0]), 2.0))
    else:
        MAA = 0
        MBA = -1 / math.sin(state[2][0])
        MCA = state[1][0] * (math.cos(state[2][0]) / pow(math.sin(state[2][0]), 2.0))

    if sel_side_laser == 0:
        MAB = -1 / math.sin(state_side)
        MBB = 0
        MCB = -(A - state[0][0]) * (math.cos(state_side) / pow(math.sin(state_side), 2.0))
    elif sel_side_laser == 1:
        MAB = 0
        MBB = 1 / math.cos(state_side)
        MCB = -(B - state[1][0]) * (math.cos(state_side) / pow(math.sin(state_side), 2.0))
    elif sel_side_laser == 2:
        MAB = -1 / math.sin(state_side)
        MBB = 0
        MCB = state[0][0] * (math.cos(state_side) / pow(math.sin(state_side), 2.0))
    else:
        MAB = 0
        MBB = 1 / math.cos(state_side)
        MCB = state[1][0] * (math.sin(state_side) / pow(math.cos(state_side), 2.0))

    Ht = np.array([[MAA, MBA, MCA, 0],
                   [MAB, MBB, MCB, 0],
                   [0, 0, 0, 1]])

    return Ht


########################################################
# Implementation of Kalman Filter (xtemp is placeholder)
########################################################


# Xt+1 = FXt + GU + W // X is before observation -
#def next_state_prior(F, X_posterio, G, U, W):
#    linearize_X = np.dot(F, X_posterio)
#    linearize_input = np.dot(G, U)
#    x_next_mean = np.add(np.add(linearize_X, linearize_input), W)
#    return x_next_mean


# Ouput next_P is before observation
# input P is posteria. P is variance of the state estimation X
# input F is linearization factor
# input Q is Variance of the noise for state estimation
#def next_state_P(F, P, Q):
#    # P = FP(F.transpose) + Q
#    next_P = np.add(np.dot(np.dot(F, P), F.transpose()), Q)
#    return next_P


# Output is next state posteri
# X_prior
# K is kalman gain
# Z is output from simulation 
# H is linearization of the 
#def next_state_posterior(X_prior, K, Z, H):
#    return np.add(X_prior, np.dot(K, np.substract(Z, np.dot(H, X_prior))))


# calculate Kalman gain
# Output Kalman Gain
# P is the posteri variance
# H is the linearization jacobian for sensors output
# R is the variance of the noise
#def kalman_gain(P, H, R):
#    K = np.dot(np.dot(P, H.transpose()), np.linalg.inv(np.add(np.dot(np.dot(H, P), H), R)))
#########################################################################
################## PLoting codes########################
# a and b must be the same size
def list_substraction(a,b):
    result = []
    for i in range(len(a)):
        result.append((a[i] - b[i]))
    return result

# take absolute of the lists
def list_abs(a):
    abs_a = []
    for i in a:
        abs_a.append(abs(i))
    return abs_a

# return lists of p00, p11, p22, p33
def extract_P_to_list(pk):
    p00 = []
    p11 = []
    p22 = []
    p33 = []
    for i in pk:
       p00.append(i[0][0])
       p11.append(i[1][1])
       p22.append(i[2][2])
       p33.append(i[3][3])

    return p00,p11,p22,p33 


# taking all P and X and plot them
def plot_all_graphs(x_estimates,P_estimates):
    x_x = []
    x_y = []
    x_delta = []
    x_delta_dot = []
    
    for i in x_estimates:
        x_x.append(i[0][0])
        x_y.append(i[1][0])
        x_delta.append(i[2][0])
        x_delta_dot.append(i[3][0])

    sim_state_file = open("simulation_states",'r')
    sim_x = []
    sim_y = []
    sim_delta = []
    sim_delta_dot = []

    for line in sim_state_file:
        if line == 'x y theta theta_dot\n' :
           pass
        else: 
           data = line.split(',')
           sim_x.append(float(data[0]))
           sim_y.append(float(data[1]))
           sim_delta.append(float(data[2]))
           sim_delta_dot.append(float(data[3]))

    x_diff = list_substraction(sim_x,x_x)
    y_diff = list_substraction(sim_y,x_y)
    delta_diff = list_substraction(sim_delta, x_delta)
    delta_dot_diff = list_substraction(sim_delta, x_delta_dot)
    
    
    p00,p11,p22,p33 = extract_P_to_list(P_estimates)
    x_axe = [0.0]
    
    for i in range(len(x_estimates) - 1): 
        x_axe.append(x_axe[i] + 0.1)
   
    plt.xlabel("time")
    plt.ylabel("values of variance P1")
    plt.plot(x_axe, p00)
    plt.show()
    
    plt.ylabel("values of variance P2")
    plt.plot(x_axe, p11)
    plt.show()

    plt.ylabel("values of variance P3")
    plt.plot(x_axe, p22)
    plt.show()

    plt.ylabel("values of variance P4")
    plt.plot(x_axe, p33)
    plt.show()
    
    plt.ylabel(' x difference ')
    plt.plot(x_axe, list_abs(x_diff))
    plt.show()
   
    plt.ylabel(' y difference ')
    plt.plot(x_axe, list_abs(y_diff))
    plt.show()

    plt.ylabel(' delta difference ')
    plt.plot(x_axe, list_abs(delta_diff))
    plt.show()

    plt.ylabel('delta dot difference')
    plt.plot(x_axe, list_abs(delta_dot_diff))
    plt.show()
    sim_state_file.close()


def main():
    print('hi')
    myfile = open("simulation_data.txt", 'r')  # file to read
    state_file_2 = open('EKF_states', 'w')  # file to write states for comparison
    state_file_2.write("x y theta theta_dot\n")
    global Fk
    global Gk
    global Hk
    global Pk
    global xPost
    global x_best
    global id_mat
    global data
    global K
    global PPost
    for line in myfile:
        if line == 'pwml   pwmr   d1   d2   mx   my   gyro   timestamp\n':
            pass
        else:
            data = line.split(',')
            pmwL.append(float(data[0]))
            pmwR.append(float(data[1]))
            d1.append(float(data[2]))
            d2.append(float(data[3]))
            mx.append(float(data[4]))
            my.append(float(data[5]))
            gyro.append(float(data[6]))
            timestamp.append(float(data[7].rstrip('\n')))

    inputVal = np.array([pmwL, pmwR])
    outputVal = np.array([d1, d2, gyro])
    inputVal = inputVal.transpose()
    outputVal = outputVal.transpose()
    state_file_2.write(str(round(x_best[0][0], 2)) + ', ' + str(round(x_best[1][0], 2)) + ', '
                       + str(round(x_best[2][0], 2)) + ',' + str(round(x_best[3][0], 2)) + "\n")

    for i in range(inputVal.shape[0]):
        Fk = f_update(x_best, inputVal[i])
        Gk = g_update(x_best)
        wl = pwm_to_w_velocity(inputVal[i][0])
        wr = pwm_to_w_velocity(inputVal[i][1])
        ut = np.array([[wl,wr]])
        xPri = np.add(np.dot(Fk, x_best), np.dot(Gk, ut.T))
        x_best = xPri
        #if x_best[2][0] < 0:  # Correct orientation for below 0 degrees and above 360 degrees
        #    x_best[2][0] = x_best[2][0] + math.pi * 2
        #elif x_best[2][0] >= math.pi * 2:
        #    x_best[2][0] = x_best[2][0] - math.pi * 2
        PPri = np.add(np.dot(np.dot(Fk, np.array(Pk)), Fk.T), Qk)
        Pk = PPri
        zk = np.array([outputVal[i]])
        Hk = h_update(x_best)
        K = np.dot(np.dot(Pk, Hk.T), np.linalg.inv(np.add(np.dot(np.dot(Hk, Pk), Hk.T), Rk)))
        xPost = (np.add(x_best, np.dot(K, (np.subtract(zk.T, np.dot(Hk, x_best))))))
        PPost = np.dot(np.subtract(id_mat, np.dot(K, Hk)), Pk)
        x_best = xPost
       # if x_best[2][0] < 0:  # Correct orientation for below 0 degrees and above 360 degrees
        #    x_best[2][0] = x_best[2][0] + math.pi * 2
       # elif x_best[2][0] >= math.pi * 2:
        #    x_best[2][0] = x_best[2][0] - math.pi * 2
        Pk = PPost
        state_file_2.write(str(round(x_best[0][0]+536, 2)) + ', ' + str(round(x_best[1][0], 2)) + ', '
                           + str(round(x_best[2][0], 2)) + ',' + str(round(x_best[3][0], 2)) + "\n")
        x_estimates.append(x_best.tolist())
        P_estimates.append(Pk.tolist())
    
    # x_estimates is a list of 4 by 1 matrices, P_estimates is a list of 4 by 4 matrices
    #plot_all_graphs(x_estimates, P_estimates)
    #print(x_estimates)
    #print(P_estimates)
    state_file_2.close()
    myfile.close()


if __name__ == "__main__":
    main()
