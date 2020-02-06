import numpy as np
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
        print('hey')
    else:
        data = line.split(',')
        pmwL.append(data[0])
        pmwR.append(data[1])
        d1.append(data[2])
        d2.append(data[3])
        mx.append(data[4])
        my.append(data[5])
        gyro.append(data[6])
        timestamp.append(data[7].rstrip('\n'))


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
x = np.array([])      #State
Pk = np.array([])     #Covariance Matrix
Fk = np.array([])     #Prediction Matrix (from linearization)
Hk = np.array([])     #Sensor Model (from linearization)
Qk = np.array([])     #Environmental Error (from experimentation)
Rk = np.array([])     #Sensor Noise (from experimentation)
K = np.array([])      #Kalman Gain
zk = np.array([])     #Sensor Reading
xtemp = np.array([])  #Posterior State
Ptemp = np.array([])  #Posterior Covariance

#Implementation of Kalman Filter (xtemp is placeholder)
for i in inputVal:
    xtemp = x[i] + 1
    Ptemp = Fk*Pk[i]*Fk.transpose() +Qk
    K = Ptemp*Hk.transpose()*np.invert(Hk*Ptemp*Hk.transpose()+Rk)
    x[i+1] = xtemp + K*(zk - Hk*xtemp)
    Pk[i+1] = Ptemp - K*Hk*Ptemp



print(inputVal)
myfile.close()
