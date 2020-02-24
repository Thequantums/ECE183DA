import numpy as np
import sys

# state class
class S:
    def __init__(self,x,y):
        self.x = x
        self.y = y

# Action class, there are 5 actions the robot can take. but we only consider 4 actions excluding "still" when finding optimal policy
class A:
    stay = 0
    left = 1
    right = 2
    up = 3
    down = 4

#global variable are constant through out the program
L, H, sizeof_action = 5, 6, 5;
pe = 0.25  #error probability in fact the robot has equal error probability of slipping to any other 4 states.ex: robot wants to move left, but it has the same probability of moving left as to other 3 directions
gamma = 0.9 # discount factor
sizeof_x = L
sizeof_y = H
sizeof_state = sizeof_x * sizeof_y
T_matrix = np.zeros((sizeof_x, sizeof_y, sizeof_action, sizeof_x, sizeof_y)) # (3 dimensional table: S,A,S' ) transitional probability
reward = np.zeros((sizeof_x, sizeof_y)) # reward array which only depends on current state (weired), only 1 dimension
all_state = []  #This variable helps us traverse through all state with index i from 0 to 29. instead of traversing by x and y all the time.
#Very important idea Note: i assume (0,0) is state 0, (1,0) is state 1 and so on till (4,5) is state 29. helpfull for indexing through alogorithm.
for y in range(sizeof_y):
    for x in range(sizeof_x):
        all_state.append(S(x,y))
##########################################

def init_T(T_matrix, Pe):
    for y in range(sizeof_y):
        for x in range(sizeof_x):
            for j in range(sizeof_action):
                for y_tr in range(sizeof_y):
                    for x_tr in range(sizeof_x):
                        if ((y_tr == 1 or y_tr == 3) and (x_tr == 1 or x_tr == 2)):
                            T_matrix[x][y][j][x_tr][y_tr] = 0  #probability of transition to yellow blocks is zero
                        elif(j == 0):
                            T_matrix[x][y][j][x_tr][y_tr] = 1 #probability of choosing to staying still is 1
                        else:
                            T_matrix[x][y][j][x_tr][y_tr] = Pe #probability of going any direction is pe regardless of action. given by spec.
    return T_matrix

def init_R(reward):
    for y in range(sizeof_y):
        for x in range(sizeof_x):
            if(y == 0 and x == 2):
                reward[x][y] = 10
            elif(y == 2 and x == 2):
                reward[x][y] = 1
            elif(x == 4):
                reward[x][y] = -100
    return reward


## QUESTION: 2A
def T(S,A,S_tran):
    return T_matrix[int(S.x)][int(S.y)][int(A)][int(S_tran.x)][int(S_tran.y)]

## QUESTION: 2B
def R(S):
    return reward[S.x][S.y]

## QUESTION: 3A
def init_policy(pi):
    #policy going left = 1 for all states
    for y in range(sizeof_y):
        for x in range(sizeof_x):
            pi[x][y] = 1
    return pi

## QUESTION: 3B
def display_policy(pi):
    for y in range(sizeof_y):
        for x in range(sizeof_x):
            if (pi[x][y] == 0):
                policy = 'stay'
            elif(pi[x][y] == 1):
                policy = 'left'
            elif(pi[x][y] == 2):
                policy = 'right'
            elif(pi[x][y] == 3):
                policy = 'up'
            else:
                policy = 'down'
            print("X= " + str(x) + ", Y= " + str(y) + ", policy: " + policy)

def set_policy_each_state(pi, S, action):
    pi[S.x][S.y] = action
    return pi

## QUESTION: 3C

def transitioned_state(state,action):
    if (action == 0):
        s = S(state.x,state.y)
    elif(action == 1):
        s = S(state.x - 1, state.y)
    elif(action == 2):
        s = S(state.x + 1, state.y)
    elif(action == 3):
        s = S(state.x, state.y + 1)
    elif(action == 4):
        s = S(state.x, state.y - 1)
    #bouding the state from going off map
    if (s.x > 4):
        s.x = 4
    if (s.x < 0):
        s.x = 0
    if (s.y > 5):
        s.y = 5
    if (s.y < 0):
        s.y = 0
    # if about to hit obstacles, stay where you are
    if ((s.y == 1 or s.y == 3) and (s.x == 1 or s.x == 2)):
        s.x = state.x
        s.y = state.y
    return s

def cant_move(state, tran_state):
    if ((state.x == tran_state.x) and (state.y == tran_state.y)):
        return 1
    else:
        return 0

def obstatcle_state(i):
    if (i==6 or i==7 or i==16 or i==17):
        return 1
    return 0


# this is for each policy: getting the value for all states. it converges less than 30 iteration
def compute_V_pi(pi):
    V_pi = np.zeros((sizeof_x,sizeof_y))
#    V_pi_new = np.zeros((sizeof_x,sizeof_y))
#    V_pi_old = np.zeros((sizeof_x,sizeof_y))
    list_V_pi = []
    accum = 0
    #only right action first
    for it in range(1000):
        #reinitialize everytime to getting new values
        V_pi_new = np.zeros((sizeof_x,sizeof_y))
        V_pi_old = np.zeros((sizeof_x,sizeof_y))
        for i in range(sizeof_state):
            accum = 0 # each state has different accumulator
            #if the index i is at yellow block, skip it to next one.
            if (obstatcle_state(i) == 0):
                for j in range(5):
                    #The J represent 5 possible actions the robot can take under one presribed action from policy.
                    s = transitioned_state(all_state[i],j) #ended up state from current state
                    # this if condition is used to set transistional probaility to 0 if the robot is prescribed by policy to go off map or go to yellow blocks
                    if (pi[all_state[i].x][all_state[i].y] != 0 and cant_move(all_state[i],s)):
                        transitonal_prob = 0
                    else:
                        transitonal_prob = T(all_state[i],pi[all_state[i].x][all_state[i].y], s)
                    accum = accum + transitonal_prob * (R(all_state[i]) + gamma * V_pi[s.x][s.y])
                    #print(str(j) + ' ' + str(s.x) + ' ' + str(s.y) + ' ' + str(transitonal_prob) + ' ' + str(accum))
            #else:
                #print("Obstacle")
            #updating V for each state
            V_pi_new[all_state[i].x][all_state[i].y] = accum # sum of expected rewards for each state under the prescibed policy
            #print('\n')
        # V_pi_old and V_pi_new are compared to see if it converges, exit loop
        V_pi_old = V_pi
        V_pi = V_pi_new #set V_pi to newly founded V_pi
        display_V_pi(V_pi)
        #this below code is just to compute if the V new and V old are not much different, then exit the loop. it usually exit under 30 iteration
        count = 0
        for c in range(sizeof_state):
            count = count + (V_pi_new[all_state[c].x][all_state[c].y] - V_pi_old[all_state[c].x][all_state[c].y])
        print((abs(count)/sizeof_state))
        if ( (abs(count)/sizeof_state) < 0.001 ):
            print(it)
            break;
    return V_pi

## QUESTION: 3d
#basically give best policy under one iteration using V_pi similar function to above
#note we are discarding the possible optimal policy of staying still which Action = 0
def opt_policy_one_step(V_pi):
    List_pi = [] #list to hold V_pi for each state under various policies, this list contains 4 values
    max_policy = np.zeros((sizeof_x,sizeof_y)) #max_policy for all states at only one step
    accum = 0
    for i in range(sizeof_state):
        del List_pi[:] #clearing list
        for policy in range(4):
            accum = 0
            #if the index i is at yellow block, skip it to next one.
            if (obstatcle_state(i) == 0):
                #if choose to move, still can have 5 possible actions leading to 5 possible ended-up states
                for j in range(5):
                    #The J represent 5 possible actions the robot can take under one presribed action from policy.
                    s = transitioned_state(all_state[i],j)
                    # this if condition is used to set transistional probaility to 0 if the robot is prescribed by policy to go off map or go to yellow blocks
                    if ((policy + 1) != 0 and cant_move(all_state[i],s)):
                        transitonal_prob = 0
                    else:
                        transitonal_prob = T(all_state[i],policy+1, s)
                    accum = accum + transitonal_prob * (R(all_state[i]) + gamma * V_pi[s.x][s.y])
                    #print(str(j) + ' ' + str(s.x) + ' ' + str(s.y) + ' ' + str(transitonal_prob) + ' ' + str(accum))
            List_pi.append(accum)
        #print("List of accum for each state: ")
        #print(List_pi)
        #THis bellow code is used to compare differnt value from different policies in each state and get the best policy for that state
        max = List_pi[0]
        max_A = 0
        #print(List_pi)
        for m in range(len(List_pi)):
            if max < List_pi[m]:
                max = List_pi[m]
                max_A = m
        max_A = max_A + 1   #because discard the a = 0 (stand still)
        #print(max_A)
        max_policy[all_state[i].x][all_state[i].y] = max_A
    #display_policy(max_policy)
    return max_policy

## QUESTION: 3e
def compute_opt_policy():
    pi = np.zeros((sizeof_x, sizeof_y))
    pi = init_policy(pi) #start by all going left (baseline policy)
    V_pi = np.zeros((sizeof_x,sizeof_y)) #initialize all zeros
    V_pi_new = np.zeros((sizeof_x,sizeof_y))
    accum = 0
    for it in range(100):
        #For every value iterations, go back to check best policy and update the policy to prescribe for the next three value iterations
        for t in range(3):
            V_pi_new = np.zeros((sizeof_x,sizeof_y))
            V_pi_old = np.zeros((sizeof_x,sizeof_y))
            for i in range(sizeof_state):
                accum = 0
                #if the index i is at yellow block, skip it to next one.
                if (obstatcle_state(i) == 0):
                    #The J represent 5 possible actions the robot can take under one presribed action from policy.
                    for j in range(5):
                        s = transitioned_state(all_state[i],j)
                        # this if condition is used to set transistional probaility to 0 if the robot is prescribed by policy to go off map or go to yellow blocks
                        if (pi[all_state[i].x][all_state[i].y] != 0 and cant_move(all_state[i],s)):
                            transitonal_prob = 0
                        else:
                            transitonal_prob = T(all_state[i],pi[all_state[i].x][all_state[i].y], s)
                        accum = accum + transitonal_prob * (R(all_state[i]) + gamma * V_pi[s.x][s.y])
                        #print(str(j) + ' ' + str(s.x) + ' ' + str(s.y) + ' ' + str(transitonal_prob) + ' ' + str(accum))
                #else:
                    #print("Obstacle")
                #updating V for each state
                V_pi_new[all_state[i].x][all_state[i].y] = accum
                #print('\n')
            V_pi_old = V_pi
            V_pi = V_pi_new
        #update policy for every three value updates
        pi = opt_policy_one_step(V_pi)
        #break loops if V converges
        count = 0
        for c in range(sizeof_state):
            count = count + (V_pi_new[all_state[c].x][all_state[c].y] - V_pi_old[all_state[c].x][all_state[c].y])
        #print((abs(count)/sizeof_state))
        if ( (abs(count)/sizeof_state) < 0.001 ):
            print("ended iteration: " + str(it*3))
            break;
    return pi

#Very important idea Note: i assume (0,0) is state 0, (1,0) is state 1 and so on till (4,5) is state 29. helpfull for indexing through alogorithm. displaying the values for all grids in a 6*5 matrix
def display_V_pi(V_pi):
    for y in range(sizeof_y):
        for x in range(sizeof_x):
            buffer = str(round(V_pi[x][sizeof_y-y-1],2)) + ", "
            sys.stdout.write(buffer)
        sys.stdout.write("\n")

def main():
    init_T(T_matrix, pe)
    init_R(reward)
    #the policy for all states
    policy = np.zeros((sizeof_x, sizeof_y))
    policy = init_policy(policy) #going left
    display_policy(policy)
    V_pi = compute_V_pi(policy)
    display_V_pi(V_pi)
    #max_po = opt_policy_one_step(V_pi)
    #display_policy(max_po)
    #V_pi = np.zeros((sizeof_x,sizeof_y))
    max_policy = compute_opt_policy()
    display_policy(max_policy)
if __name__ == '__main__':
    main()
