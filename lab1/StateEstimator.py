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

x = [[250,250]]                  #State     (each state of size n)
Pk = [[1,0],[0,1]]               #Covariance Matrix    (each element of size nxn)
Fk = np.array([[0,0],[0,0]])     #Prediction Matrix (from linearization) (size nxn)
Hk = np.array([[1,0],[0,1]])     #Sensor Model (from linearization)
Qk = np.array([[0,0],[0,0]])     #Environmental Error (from experimentation)
Rk = np.array([[0,0],[0,0]])     #Sensor Noise (from experimentation)
K = np.array([])                 #Kalman Gain
zk = np.array([1,2])             #Sensor Reading
xPost = np.array([])             #Posteriori State
PPost = np.array([])             #Posteriori Covariance
##########################################################
# State Dynamics Function (State+Input=>Posteriori State)
##########################################################
#Takes a state at one instant and a set of PWM vals at that instant. Should output as a list with size=x.size

def StateDyn(x,inVals):
    xout = np.array([])
   ####State Dynamics Go Here (erase my garbage test code
    if inVals[0]>0:
        xout = np.array(x) + np.array([1,1])
    else:
        xout = np.array(x) - np.array([1,1])
    ####
    return xout.tolist()


########################################################
#Implementation of Kalman Filter (xtemp is placeholder)
########################################################

for i in range(inputVal.shape[0]-1):
    xPost = StateDyn(x[i],inputVal[i])
    PPost = np.dot(Fk,np.dot(np.array(Pk[i]),Fk.T)) + Qk
    K = np.dot(PPost,np.dot(Hk.T,np.invert(np.dot(Hk,np.dot(PPost,Hk.T)+Rk))))
    x.append(((xPost + np.dot(K,(zk-np.dot(Hk,xPost)))).reshape(2)).tolist())
    Pk.append(PPost - np.dot(K,np.dot(Hk,PPost)))
print(x)
myfile.close()
