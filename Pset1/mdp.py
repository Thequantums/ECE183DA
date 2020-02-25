#!/usr/bin/python3
import numpy as np
import sys
import time
import argparse

# state class
class S:
    def __init__(self, x, y):
        self.x = x
        self.y = y


# Action class, there are 5 actions the robot can take
class A:
    stay = 0
    left = 1
    right = 2
    up = 3
    down = 4


# Global variable are constant through out the program
# error probability in fact the robot has equal error probability of slipping to any other 4 states.ex:
# robot wants to move left, but it has the same probability of moving left as to other 3 directions
L, H, sizeof_action = 5, 6, 5  # Length, Height, Na
pe = 0.01  # Error Probability
gamma = 0.9  # Discount factor
threshold = .00001  # Threshold for convergence
sizeof_x = L
sizeof_y = H
sizeof_state = sizeof_x * sizeof_y  # Ns

# (3 dimensional table: S,A,S' ) transitional probability
T_matrix = np.zeros((sizeof_x, sizeof_y, sizeof_action, sizeof_x, sizeof_y))

# Reward array which only depends on current state, only 1 dimension
reward = np.zeros((sizeof_x, sizeof_y))

# This variable helps us traverse through all state with index i from 0 to 29. instead of traversing by x and y
# all the time. Very important Note:(0,0) is state 0, (1,0) is state 1, (0, 1) is state 5 and so on till (4,5) is
# state 29. helpful for indexing in algorithm.
all_state = []
for y in range(sizeof_y):
    for x in range(sizeof_x):
        all_state.append(S(x, y))


##########################################

# QUESTION: 1C and 2A
# Initializes all possible transitions, much work was done to get these values correct so that no further
# work would need to be done in other sections to account for all probabilistic behaviour
def init_T(T_matrix, Pe):
    for y in range(sizeof_y):
        for x in range(sizeof_x):
            for j in range(sizeof_action):
                for y_tr in range(sizeof_y):
                    for x_tr in range(sizeof_x):
                        # probability of transitioning to yellow blocks is zero so remains zero
                        if (y_tr == 1 or y_tr == 3) and (x_tr == 1 or x_tr == 2):
                            continue
                        # probability of staying still if choosing to not move is 1
                        elif j == 0 and x == x_tr and y == y_tr:
                            T_matrix[x][y][j][x_tr][y_tr] = 1
                        # probability of moving if choosing to not move is impossible so remains zero
                        elif j == 0:
                            continue
                        # probability of moving to intended direction that is within bounds is 1 - Pe + Pe/4
                        elif (j == 1 and x - 1 == x_tr and y == y_tr) or \
                                (j == 2 and x + 1 == x_tr and y == y_tr) or \
                                (j == 3 and x == x_tr and y + 1 == y_tr) or \
                                (j == 4 and x == x_tr and y - 1 == y_tr):
                            T_matrix[x][y][j][x_tr][y_tr] = 1 - 3 * Pe / 4
                        # probability of moving to unintended direction that is within bounds is Pe - Pe/4
                        elif (x - 1 == x_tr and y == y_tr) or \
                                (x + 1 == x_tr and y == y_tr) or \
                                (x == x_tr and y + 1 == y_tr) or \
                                (x == x_tr and y - 1 == y_tr):
                            T_matrix[x][y][j][x_tr][y_tr] = Pe / 4
                        # probability of staying still in non-corner/non-corridor space when moving towards
                        # a wall/obstacle is 1 - Pe + Pe/4
                        elif (x == x_tr and y == y_tr) and \
                                ((j == 1 and ((x == 0 and 1 < y < 5 and y != 3) or (x == 3 and (y == 3 or y == 1)))) or
                                 (j == 2 and x == 4 and 0 < y < 5) or
                                 (j == 3 and 0 < x < 4 and y == 5) or
                                 (j == 4 and ((x == 3 and y == 0) or ((x == 1 or x == 2) and y == 4)))):
                            T_matrix[x][y][j][x_tr][y_tr] = 1 - 3 * Pe / 4
                        # probability of staying still when moving away from a wall/obstacle is Pe - Pe/4
                        elif (x == x_tr and y == y_tr) and \
                                (((x == 0 and 1 < y < 5 and y != 3) or (x == 3 and (y == 3 or y == 1))) or
                                 (x == 4 and 0 < y < 5) or
                                 (0 < x < 4 and y == 5) or
                                 ((x == 3 and y == 0) or ((x == 1 or x == 2) and y == 4))):
                            T_matrix[x][y][j][x_tr][y_tr] = Pe / 4
                        # probability of staying still in a corner/corridor while moving towards
                        # wall/obstacle is 1 - Pe + Pe/2
                        elif (x == x_tr and y == y_tr) and \
                                ((j == 1 and x == 0) or
                                 (j == 2 and ((x == 0 and (y == 1 or y == 3)) or x == 4)) or
                                 (j == 3 and (((x == 1 or x == 2) and (y == 0 or y == 2)) or y == 5)) or
                                 (j == 4 and (((x == 1 or x == 2) and y == 2) or y == 0))):
                            T_matrix[x][y][j][x_tr][y_tr] = 1 - Pe / 2
                        # probability of staying still in a corner while moving away from wall is Pe - Pe/2
                        elif x == x_tr and y == y_tr and (x == 0 or x == 4 or y == 5 or y == 0):
                            T_matrix[x][y][j][x_tr][y_tr] = Pe / 2
                        # all other motions(pretty much teleportation) are impossible so remain zero
    return T_matrix


# Returns transition probability given initial state, action, and next state
def T(s, a, s_tran):
    return T_matrix[int(s.x)][int(s.y)][int(a)][int(s_tran.x)][int(s_tran.y)]


# Question: 2B
# Initializes Reward array
def init_R(reward):
    for y in range(sizeof_y):
        for x in range(sizeof_x):
            if y == 0 and x == 2:
                reward[x][y] = 10
            elif y == 2 and x == 2:
                reward[x][y] = 1
            elif x == 4:
                reward[x][y] = -100
    return reward


# Returns reward given initial state (only needs state since reward is only determined by initial state)
def R(s):
    return reward[s.x][s.y]


# QUESTION: 3A
# Initialized policy to take left in all states
def init_policy(pi):
    # policy going left = 1 for all states
    for y in range(sizeof_y):
        for x in range(sizeof_x):
            pi[x][y] = 1
    return pi


# QUESTION: 3B
# Displays input policy in a grid format
def display_policy_ingridworld(pi):
    for y in range(sizeof_y):
        new_y = sizeof_y - y - 1
        for x in range(sizeof_x):
            # Obstacles are not important so fill region with -----
            if (x == 1 or x == 2) and (new_y == 1 or new_y == 3):
                buffer = "-----  "
                sys.stdout.write(buffer)
                continue
            if pi[x][new_y] == 0:
                policy = 'stay '
            elif pi[x][new_y] == 1:
                policy = 'left '
            elif pi[x][new_y] == 2:
                policy = 'right'
            elif pi[x][new_y] == 3:
                policy = 'up   '
            else:
                policy = 'down '
            buffer = policy + '  '
            sys.stdout.write(buffer)
        sys.stdout.write("\n")


# Helper function: Checks to see if current state is obstacle state
def obstacle_state(i):
    return i == 6 or i == 7 or i == 16 or i == 17


# Helper function: Checks for convergence using norm of infinity (updated)
def convergence_test_v(v1, v2):
    global threshold
    max_diff = 0
    # Double loop finds the difference between elements and selects maximum difference
    for x in range(sizeof_x):
        for y in range(sizeof_y):
            diff = abs(v1[x][y] - v2[x][y])
            if diff > max_diff:
                max_diff = diff
    # Check for Convergence
    return max_diff < threshold


# QUESTION: 3C
# Evaluation of V for a given policy (updated)
def compute_v_pi(pi):
    global all_state
    # Initialize V to all zeros
    v_pi = np.zeros((sizeof_x, sizeof_y))
    # Main Loop
    while 1:
        # Initialize array that will hold all computed values for this iteration
        v_pi_new = np.zeros((sizeof_x, sizeof_y))
        # For every state
        for i in range(sizeof_state):
            # Don't do anything for V(s) where s is an obstacle
            if obstacle_state(i) == 1:
                continue
            # Will hold total for expected sum
            value = 0
            # For every state, loop through all possible future states
            for j in range(sizeof_state):
                # The actual evaluation of pi using the Bellman Backup
                value += T(all_state[i], pi[all_state[i].x][all_state[i].y], all_state[j]) * (
                            R(all_state[i]) + gamma * v_pi[all_state[j].x][all_state[j].y])
            # Fill in entry of v_pi_new
            v_pi_new[all_state[i].x][all_state[i].y] = value
        # Check for convergence
        if convergence_test_v(v_pi, v_pi_new):
            break
        # If not converged update V and iterate again
        v_pi = v_pi_new
    return v_pi


# QUESTION: 3D
# Greedy one step lookahead improvement on policy (Bellman Backup) (updated)
def improve_pi(v):
    global all_state
    # Create array to hold improved policy
    pi_improved = np.zeros((sizeof_x, sizeof_y))
    # Loop over all states
    for i in range(sizeof_state):
        # Ignore obstacle states
        if obstacle_state(i):
            continue
        # Array to hold expected values of all actions
        action_values = np.zeros(sizeof_action)
        # For every state also loop through every possible action
        for policy in range(sizeof_action):
            # For every state action combo, also loop through every possible next state
            for j in range(sizeof_state):
                # The actual Bellman Backup that sums the expected values for all actions
                action_values[policy] += T(all_state[i], policy, all_state[j]) * (
                            R(all_state[i]) + gamma * v[all_state[j].x][all_state[j].y])
        # Choose the action that increases V the most
        pi_improved[all_state[i].x][all_state[i].y] = np.argmax(action_values)
    return pi_improved


# QUESTION: 3E
# Policy Iteration Algorithm (updated)
def policy_iteration():
    # Start the timer
    start = time.perf_counter()  # Question 3f
    # Initialize policy to all lefts
    pi = np.zeros((sizeof_x, sizeof_y))
    pi = init_policy(pi)

    # Main Loop
    while 1:
        # Evaluation Step (Evaluate V under current policy)
        v_opt = compute_v_pi(pi)
        # Improvement Step (Improve policy under greedy one step lookahead using V under current policy)
        improved_pi = improve_pi(v_opt)
        # If policy did not change then optimal policy found
        if np.array_equal(improved_pi, pi):
            break
        # Otherwise update the current policy and run next iteration
        pi = improved_pi
    # End timer and get time elapsed
    end = time.perf_counter()  # Question 3f
    policy_iteration_time = end - start  # Question 3f
    return pi, v_opt, policy_iteration_time


# QUESTION: 4A
# Value Iteration Algorithm (updated)
def value_iteration():
    global all_state
    # Start the timer
    start = time.perf_counter()  # Question 4c
    # Initialize V to all zeros
    v_opt = np.zeros((sizeof_x, sizeof_y))

    # Main Loop
    while 1:
        # Array will hold current iterations values
        v_new = np.zeros((sizeof_x, sizeof_y))
        # For all states
        for i in range(sizeof_state):
            # Array will hold values for different actions so we can maximize
            action_values = np.zeros(sizeof_action)
            # For every state loop through every action
            for policy in range(sizeof_action):
                # For every state action combo, loop through every possible next state
                for j in range(sizeof_state):
                    # The actual Bellman Backup computation
                    action_values[policy] += T(all_state[i], policy, all_state[j]) * (
                                R(all_state[i]) + gamma * v_opt[all_state[j].x][all_state[j].y])
            # Maximize value of V(s)
            v_new[all_state[i].x][all_state[i].y] = np.max(action_values)
        # Check for convergence
        if convergence_test_v(v_opt, v_new):
            break
        # If did not converge, update V and run next iteration
        v_opt = v_new
    # Now that you have optimal V, extract optimal policy from it
    extracted_pi = improve_pi(v_opt)
    # End timer and get time elapsed
    end = time.perf_counter()  # Question 4c
    value_iteration_time = end - start  # Question 4c
    return extracted_pi, v_opt, value_iteration_time


# Helper Function: Prints Value function in gird for format
# Note:(0,0) is state 0, (1,0) is state 1 and so on till (4,5) is state 29.
def display_v_pi(v_pi):
    for y in range(sizeof_y):
        for x in range(sizeof_x):
            # Obstacles are unimportant so print as -----
            if (x == 1 or x == 2) and (y == 2 or y == 4):
                buffer = " -----  "
                sys.stdout.write(buffer)
                continue
            buffer = "%.2f" % v_pi[x][sizeof_y - y - 1] + ", "
            # For alignment
            if v_pi[x][sizeof_y - y - 1] > 0:
                buffer = " " + buffer
            # For alignment
            if abs(v_pi[x][sizeof_y - y - 1]) < 10:
                buffer = " " + buffer
            sys.stdout.write(buffer)
        sys.stdout.write("\n")


def findTrajectory(initialState,policy): #Finds the trajectory and max sum of discounted rewards given a starting point and policy
    trajectory = [initialState]     #place initial value into trajectory array
    sum_of_rewards = 0
    sumtemp = 1000
    t = 0   #initialize values
    while(True):
        currentState = trajectory[-1]   #take current state from end of array
        action = policy[currentState.x][currentState.y]     #determine action based on state and policy
        if action == 1:     #determine next state based on action
            currentState = S(currentState.x - 1 ,currentState.y)
        elif action == 2:
            currentState = S(currentState.x + 1 ,currentState.y)
        elif action == 3:
            currentState = S(currentState.x ,currentState.y + 1)
        elif action == 4:
            currentState = S(currentState.x,currentState.y - 1)
        sumtemp = sum_of_rewards    #save sum of rewards for comparison
        sum_of_rewards = sum_of_rewards + (pow(gamma,t)*R(currentState))    #calculate sum of rewards
        t = t+1
        trajectory.append(currentState)     #add current state to trajectory list
        if action == 0 and abs(sumtemp-sum_of_rewards) > threshold: #check to see if staying stationary has had any relevance
            break
    return [trajectory,sum_of_rewards]


def listTrajectory(traj):   #Returns a list of coordinate pairs given a list of states
    listform = []
    for s in traj:
        listform.append([s.x,s.y])
    return listform


# Main Function
def main():
    paser = argparse.ArgumentParser()
    paser.add_argument('--pe', action = "store", dest="pr_e", default = 0.01, type=float,help="Error Probability")
    paser.add_argument('--gamma', action ="store", dest="Gamma", default =0.9, type=float, help="Discount Factor")
    paser.add_argument('--output', action ="store", dest="output", default ='opt', help="[opt] only displays optimal results, [all] displays all results")
    inputs = paser.parse_args()
    global pe, gamma
    pe = inputs.pr_e
    gamma = inputs.Gamma
    output_type = inputs.output

    init_T(T_matrix, pe)
    init_R(reward)
    # the policy for all states
    policy = np.zeros((sizeof_x, sizeof_y))
    policy = init_policy(policy)  # going left
    v_pi = compute_v_pi(policy)
    one_step_opt_policy = improve_pi(v_pi)
    opt_policy, opt_value, policy_iteration_time = policy_iteration()
    value_opt_policy, opt_value, values_iteration_time = value_iteration()
    policyT = findTrajectory(S(2,5),opt_policy)
    valueT = findTrajectory(S(2, 5), value_opt_policy)
    if (output_type == 'all'):
        print('Display initial policy: ')
        display_policy_ingridworld(policy)
        print('\none-step improvement on initial policy: ')
        display_policy_ingridworld(one_step_opt_policy)

    #only display optimal results
    print("\nPolicy iteration result: ")
    display_policy_ingridworld(opt_policy)
    print("CPU time for policy iteration is " + str((policy_iteration_time * pow(10, 3))) + " ms \n")
    print("Optimal Path: ",str(listTrajectory(policyT[0])), '\n', 'Max sum of discounted rewards: ', str(policyT[1]))
    print("Value iteration result: ")
    display_policy_ingridworld(value_opt_policy)
    print("CPU time for value iteration is " + str((values_iteration_time * pow(10, 3))) + " ms \n")
    print("Optimal Path: ",str(listTrajectory(valueT[0])), '\n', 'Max sum of discounted rewards: ', str(valueT[1]))


if __name__ == '__main__':
    main()
