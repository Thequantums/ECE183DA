import numpy as np
import math
myfile = open("simulation_data.txt", 'r')  # file to read

pmwL = []
pmwR = []

d1 = []
d2 = []
mx = []
my = []
gyro = []
timestamp = []
data = []
#######################################################
#Reads File into individual arrays for input and output
#######################################################
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
###################################################
#Outputs inputVal array and outputVal array
###################################################

Rw = 50
d = 1
L = 90
T = 0.1
A = 530
B = 400
###################################################
# Kalman variables
###################################################

x = [[250,250, 1,0]]                            #State     (each state of size n) (x,y,theta,thetaDot)
Pk = [[1,1,1,1],[1,1,1,1], [1,1,1,1], [1,1,1,1]]                #Covariance Matrix    (each element of size nxn)
Fk = np.array([])     #Prediction Matrix (from linearization) (size nxn)
Gk = np.array([])   #input matrix
Hk = np.array([])              #Sensor Model (from linearization)(size mxn)
Qk = np.array([[1,0,0,0],[0,1,0,0], [0,0,1,0], [0,0,0,1]] )     #Environmental Error (from experimentation)(size nxn)
Rk = np.array([[1,0,0],[0,1,0],[0,0,1]] )              #Sensor Noise (from experimentation)(size nxm)
K = np.array([])                 #Kalman Gain
zk = np.array([1,2,3,4,5])             #Sensor Reading (size m)
xPost = np.array([])             #Posteriori State
PPost = np.array([])             #Posteriori Covariance
##########################################################
#PWM->Velocity
##########################################################
def pwm_to_velocity(pwm):
    # Function takes pwm value and turns into an angular velocity which is then
    # converted into a translational velocity. Angular velocity is found using
    # experimentation where a non-linear plot is broken down into various linear
    # regions with dead zones and saturation. Experimentation can be found on lab report
    R = 50
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
##########################################################

##########################################################
# State Dynamics Function (State+Input=>Posteriori State)
##########################################################
#Takes a state at one instant and a set of PWM vals at that instant. Should output as a list with size=x.size

def StateDyn(currentState,inVals):
    xout = []
    vl = pwm_to_velocity(inVals[0])
    vr = pwm_to_velocity(inVals[1])
    L = 90
    T = 0.1
   ####State Dynamics Go Here (erase my garbage test code
    if vl != vr:  # If the robot is following a curved path
        r_curve = L * (vl + vr) / (2 * (vr - vl))  # calculate radius of curvature
        w_curve = (vr - vl) / L  # calculate angular velocity
        ccx = currentState[0] - r_curve * math.sin(currentState[2])  # calculate center of curvature (x coord)
        ccy = currentState[1] + r_curve * math.cos(currentState[2])  # calculate center of curvature (y coord)
        x_next = (currentState[0] - ccx) * math.cos(w_curve * T) - (currentState[1] - ccy) * math.sin(w_curve * T) + ccx  # calc next x coord
        y_next = (currentState[0] - ccx) * math.sin(w_curve * T) + (currentState[1] - ccy) * math.cos(w_curve * T) + ccy  # calc next y coord
        theta_next = currentState[2] + w_curve * T  # calc next theta
    else:  # If the robot is following a straight path
        x_next = vl * T * math.cos(currentState[2]) + currentState[0]  # calc next x coord
        y_next = vl * T * math.sin(currentState[2]) + currentState[1]  # calc next y coord
        theta_next = currentState[2]  # maintain heading
    xout =[x_next,y_next,theta_next,0]
    ####
    return xout
def F_Update(state,inValues):
    global Rw
    global d
    global T
    a = Rw*(inValues[0]+inValues[1])/2
    b = Rw*(inValues[1]-inValues[0])/(2*d)
    Ft = np.array([[1,0,-a*math.sin(state[2])*T,0],[0,1,a*math.cos(state[2])*T,0],[0,0,1,0],[0,0,0,0]])
    return Ft


def G_Update(state):
    global Rw
    global d
    global T
    Gt = np.array([[Rw*math.cos(state[2])*T/2,Rw*math.cos(state[2])*T/2],[Rw*math.sin(state[2])*T/2,Rw*math.sin(state[2])*T/2],[-Rw*T/(2*d),Rw*T/(2*d)],[-Rw/(2*d),Rw/(2*d)]])
    return Gt


def H_Update(state):
    global A
    global B
    global L
    global w
    d = [0, 0, 0, 0]
    if state[2] == math.pi / 2 or state[2] == 3 * math.pi / 2:
        d[1] = 1000000
        d[3] = 1000000
    else:
        d[1] = (B - state[1]) / math.sin(state[2])
        d[3] = state[0] / math.sin(state[2] + math.pi)
    if state[2] == 0 or math.pi:
        d[0] = 1000000
        d[2] = 1000000
    else:
        d[0] = (A - state[0]) / math.cos(state[2])
        d[2] = state[0] / math.cos(state[2] + math.pi)
    for x in range(4):
        if d[x] < 0:
            d[x] = 1000000

    selD = d.index(min(d))

    if selD == 1:
        MAA = -1/math.cos(state[2])
        MBA = 0
        MCA = (L-state[0])*(math.sin(state[2])/pow(math.cos(state[2]),2.0))
        MAB = -1/math.sin(state[2])
        MBB = 0
        MCB = -(L-state[0])*(math.cos(state[2])/pow(math.sin(state[2]),2.0))
    elif selD == 2:
        MAA = 0
        MBA = -1/math.sin(state[2])
        MCA = -(w-state[1])*(math.cos(state[2])/pow(math.sin(state[2]),2.0))
        MAB = 0
        MBB = 1/math.cos(state[2])
        MCB = -(w-state[1])*(math.cos(state[2])/pow(math.sin(state[2]),2.0))
    elif selD == 3:
        MAA = -1/math.cos(state[2])
        MBA = 0
        MCA = -state[0]*(math.sin(state[2])/pow(math.cos(state[2]),2.0))
        MAB = -1/math.sin(state[2])
        MBB = 0
        MCB = state[0]*(math.cos(state[2])/pow(math.sin(state[2]),2.0))
    else:
        MAA = 0
        MBA = -1/math.sin(state[2])
        MCA = state[1]*(math.cos(state[2])/pow(math.sin(state[2]),2.0))
        MAB = 0
        MBB = 1/math.cos(state[2])
        MCB = state[1]*(math.sin(state[2])/pow(math.cos(state[2]),2.0))

    Ht = np.array([[MAA,MBA,MCA,0],[MAB,MBB,MCB,0],[0,0,0,1]])

    return Ht

########################################################
#Implementation of Kalman Filter (xtemp is placeholder)
########################################################

def main():
    for i in range(inputVal.shape[0]-1):
        zk = outputVal[i]
        Fk = F_Update(x[i],inputVal[i])
        Gk = G_Update(x[i])
        Hk = H_Update(x[i])

        xPost = np.dot(Fk,x[i])+np.dot(Gk,inputVal[i+1])
        PPost = np.dot(np.dot(Fk,np.array(Pk[i])),Fk.T) + Qk
        K = np.dot(np.dot(PPost, Hk.T),np.linalg.inv(np.dot(np.dot(Hk,PPost), Hk.T) + Rk))
        x.append((xPost + np.dot(K,(zk-np.dot(Hk,xPost)))).tolist())
        Pk.append((PPost - np.dot(np.dot(K,Hk),PPost)).tolist())
    print(x)
    print(Pk)
    myfile.close()

if __name__ == "__main__":
    main()

#####################################################################
#####################################################################
#####################################################################

# Xt+1 = FXt + GU + W // X is before observation -
def next_state_prior(F, X_posterio, G, U, W):
    linearize_X = np.dot(F, X_posterio)
    linearize_input = np.dot(G,U)
    x_next_mean = np.add(np.add(linearize_X, linearize_input), W)
    return x_next_mean

# Ouput next_P is before observation
# input P is posteria. P is variance of the state estimation X
# input F is linearization factor
# input Q is Variance of the noise for state estimation
def next_state_P(F, P, Q):
    #P = FP(F.transpose) + Q
    next_P = np.add(np.dot(np.dot(F, P), F.transpose()), Q) 
    return next_P 

#Output is next state posteri
#X_prior
# K is kalman gain
# Z is output from simulation 
# H is linearization of the 
def next_state_posterior(X_prior, K, Z, H):
    return np.add(X_prior, np.dot(K, np.substract(Z, np.dot(H, X_prior))))

#calculate Kalman gain
# Output Kalman Gain
# P is the posteri variance
# H is the linearization jacobian for sensors output
# R is the variance of the noise
def kalman_gain(P,H, R):
    K = np.dot(np.dot(P, H.transpose()), np.linalg.inv(np.add(np.dot(np.dot(H,P),H), R)))


def main():
    
    
if __name__== "__main__":
    main()    
