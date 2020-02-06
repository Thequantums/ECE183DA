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

x = [[250,250]]                  #State     (each state of size n)
Pk = [[1,1],[1,1]]               #Covariance Matrix    (each element of size nxn)
Fk = np.array([[1,0],[3,1]])     #Prediction Matrix (from linearization) (size nxn)
Hk = np.array([[2,1],[1,1]])     #Sensor Model (from linearization)
Qk = np.array([[0,0],[0,0]])     #Environmental Error (from experimentation)
Rk = np.array([[0,0],[0,0]])     #Sensor Noise (from experimentation)
K = np.array([])                 #Kalman Gain
zk = np.array([0,0])             #Sensor Reading
xtemp = np.array([])             #Posterior State
Ptemp = np.array([])             #Posterior Covariance

########################################################
#Implementation of Kalman Filter (xtemp is placeholder)
########################################################

for i in range(inputVal.shape[0]):
    xtemp = np.array(x[i])
    Ptemp = np.dot(Fk,np.dot(np.array(Pk[i]),Fk.T)) +Qk
    K = np.dot(Ptemp,np.dot(Hk.T,np.invert(np.dot(Hk,np.dot(Ptemp,Hk.T)+Rk))))
    x.append(((xtemp + np.dot(K,(zk-np.dot(Hk,xtemp)))).reshape(2)).tolist())
    Pk.append(Ptemp - np.dot(K,np.dot(Hk,Ptemp)))
print(x)



myfile.close()
