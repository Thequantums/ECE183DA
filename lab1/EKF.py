import numpy as np
import math
import paperbot_act as bot

Rw = 20
L = 85
d = L / 2
T = 0.1
A = 530
B = 400

Hk_Correction = np.array([[A],[B],[0]])

def init(Rwl = 20 ,Ll = 85 ,Tl = 0.1 ,Al = 530 ,Bl = 400):
    global Rw
    global L
    global d
    global T
    global A
    global B
    Rw = Rwl
    L = Ll
    d = L / 2
    T = Tl
    A = Al
    B = Bl


def f_update(state, in_values): #
    global Rw
    global d
    global T
    wl = bot.pwm_to_w_velocity(in_values[0])
    wr = bot.pwm_to_w_velocity(in_values[1])
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
    front_laser = [0, 0, 0, 0]  #d1
    side_laser = [0, 0, 0, 0]   #d2
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
    print('sel side'+ str(sel_side_laser))
    print('sel front' + str(sel_front_laser))
    if sel_front_laser == 0:    #fixed
        MAA = -1 / math.cos(state[2][0])
        MBA = 0
        MCA = (A - state[0][0]) * (math.sin(state[2][0]) / pow(math.cos(state[2][0]), 2.0))
    elif sel_front_laser == 1:
        MAA = 0
        MBA = -1 / math.cos(state[2][0])
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
        MAB = 1 / math.sin(state_side)
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
    else:   #fixed
        MAB = 0
        MBB = 1 / math.sin(state_side)
        MCB = state[1][0] * (math.sin(state_side) / pow(math.cos(state_side), 2.0))

    Ht = np.array([[MAA, MBA, MCA, 0],
                   [MAB, MBB, MCB, 0],
                   [0, 0, 0, 1]])
    print(MBB)

    return Ht

def EKF(x_best,Pk,inputVal, outputVal,Qk,Rk):
    x_estimates = [x_best.tolist()]     #Add initial state to estimate list
    P_estimates = [Pk.tolist()]     #Add initial covariance to estimate list
    for i in range(inputVal.shape[0]):      #loop over each input value
        Fk = f_update(x_best, inputVal[i])  #Calculate Fk update
        Gk = g_update(x_best)       #Calculate Gk update
        wl = bot.pwm_to_w_velocity(inputVal[i][0])  #get angular velocity for left wheel
        wr = bot.pwm_to_w_velocity(inputVal[i][1])  #get angular velocity for right wheel
        ut = np.array([[wl, wr]])       #build input array
        xPri = np.add((Fk @ x_best), (Gk @ ut.T))     #do calculation for Priori state
        x_best = xPri
        # if x_best[2][0] < 0:  # Correct orientation for below 0 degrees and above 360 degrees
        #    x_best[2][0] = x_best[2][0] + math.pi * 2
        # elif x_best[2][0] >= math.pi * 2:
        #    x_best[2][0] = x_best[2][0] - math.pi * 2
        PPri = np.add((Fk @ np.array(Pk) @ Fk.T), Qk)   #do calculation for Priori covariance
        Pk = PPri
        zk = np.array([outputVal[i]])   #build output array
        Hk = h_update(x_best)   #perform Hk update
        K = np.dot((Pk @ Hk.T), np.linalg.inv(np.add(np.dot(np.dot(Hk, Pk), Hk.T), Rk)))   #Calculate Kalman Gain
        print((Hk @ x_best)+Hk_Correction)
        xPost = (np.add(x_best, np.dot(K, (np.subtract(zk.T, ((Hk @ x_best)+Hk_Correction))))))    #Calculate posteriori state
        PPost = np.dot(np.subtract(np.identity(4), np.dot(K, Hk)), Pk)      #Calculate Posteriori covariance
        x_best = xPost
        # if x_best[2][0] < 0:  # Correct orientation for below 0 degrees and above 360 degrees
        #    x_best[2][0] = x_best[2][0] + math.pi * 2
        # elif x_best[2][0] >= math.pi * 2:
        #    x_best[2][0] = x_best[2][0] - math.pi * 2
        Pk = PPost
        x_estimates.append(x_best.tolist()) #add posteriori to state list
        P_estimates.append(Pk.tolist())     #add posteriori to covariance list
    print(x_estimates)
    return [x_estimates,P_estimates]