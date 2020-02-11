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
outputVal = np.array([d1, d2, mx, my, gyro])
inputVal = inputVal.transpose()
outputVal = outputVal.transpose()
###################################################
#Outputs inputVal array and outputVal array
###################################################

###################################################
# Kalman variables
###################################################

x = [[250,250, 0]]                            #State     (each state of size n) (x,y,theta)
Pk = [[0,0,0],[0,0,0], [0,0,0]]                #Covariance Matrix    (each element of size nxn)
Fk = np.array([[0,0,0],[0,0,0], [0,0,0]] )     #Prediction Matrix (from linearization) (size nxn)
Hk = np.array([[0,0,0],[0,0,0]] )              #Sensor Model (from linearization)(size mxn)
Qk = np.array([[0,0,0],[0,0,0], [0,0,0]] )     #Environmental Error (from experimentation)(size nxn)
Rk = np.array([[0,0],[0,0],[0,0]] )              #Sensor Noise (from experimentation)(size nxm)
K = np.array([])                 #Kalman Gain
zk = np.array([1,2])             #Sensor Reading (size m)
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
    xout =[x_next,y_next,theta_next]
    ####
    return xout


########################################################
#Implementation of Kalman Filter (xtemp is placeholder)
########################################################

for i in range(inputVal.shape[0]-1):
    xPost = StateDyn(x[i],inputVal[i])
    PPost = np.dot(Fk,np.dot(np.array(Pk[i]),Fk.T)) + Qk
    K = np.dot(PPost,np.dot(Hk.T,np.invert(np.dot(Hk,np.dot(PPost,Hk.T)+Rk))))
    x.append(((xPost + np.dot(K,(zk-np.dot(Hk,xPost)))).reshape(3)).tolist())
    Pk.append(PPost - np.dot(K,np.dot(Hk,PPost)))
print(x)
myfile.close()
