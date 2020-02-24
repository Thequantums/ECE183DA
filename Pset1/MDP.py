import numpy as np

L = 5
H = 6
class State:


    def __init__(self,x,y):
        self.x = x
        self.y = y


stateSpace = np.zeros((L,H))
stateSpace = stateSpace.tolist()
print(stateSpace)
actionSpace = [0, 1, 2, 3, 4]

def transprop(currentstate,action,nextstate):



x=0
for i in range(0,len(stateSpace)):
    for k in stateSpace[i]:
        for j in actionSpace:
            x=x+1

print(x)