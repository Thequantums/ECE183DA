import numpy as np
import math
import matplotlib.pyplot as plt
import EKF
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
Fk = np.array([])  # Prediction Matrix (from linearization) (size nxn)
Gk = np.array([])  # input matrix
Hk = np.array([])  # Sensor Model (from linearization)(size mxn)
K = np.array([])  # Kalman Gain  # Sensor Reading (size m)
xPost = np.array([])  # Posteriori State
PPost = np.array([])  # Posteriori Covariance

x_estimates = [x_best]
P_estimates = [Pk]



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
    state_file_2.write("x".ljust(10) + "y".ljust(10) + "theta".ljust(10) + "theta_dot".ljust(10) + "\n")
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

    EKF.init()
    [x_estimates,P_estimates] = EKF.EKF(x_best,Pk,inputVal,outputVal,Qk,Rk)

    for i in x_estimates:
        for k in i:
            state_file_2.write(str(round(k[0],3)).ljust(10,' '))
        state_file_2.write('\n')

    # x_estimates is a list of 4 by 1 matrices, P_estimates is a list of 4 by 4 matrices
    #plot_all_graphs(x_estimates, P_estimates)
    print(x_estimates)
    print(P_estimates)
    state_file_2.close()
    myfile.close()


if __name__ == "__main__":
    main()
