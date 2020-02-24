import numpy as np

class StateSpace:

    def __init__(self,H,L):
        spacetemp = []
        self.H = H
        self.L = L
        self.space = []
        for i in range(0,H):
            for k in range(1,L+1):
                spacetemp.append(k + (((H+1) * i)))
            self.space.append(spacetemp)
            spacetemp = []
        self.size = len(self.space) * len(self.space[0])


    def index(self,s):
        [x,y] = s
        return self.space[x][y]


    def isValid(self,s ):
        [x,y] = s
        return(0 <= x <= self.H-1 and 0 <= y <= self.L-1)


def isAdjacent(s,sp):
    [sx,sy] = s
    [spx,spy] = sp
    print()
    if not(sx == spx and sy == spy) and ((sx-1 <= spx <= sx+1 and sy == spy) or (sy-1 <= spy <= sy+1 and sx == spx)):
        return True
    else:
        return False


def isObstacle(O,s):
    for o in O:
        if s == o:
            return True
    return False


def reward(R,s):
    for r in R:
        if s == r[0:2]:
            return r[2]
    return 0

def transition_p(S,O,s,a,sp):
    globprop = 0.01
    [sx,sy] = s
    [spx,spy] = sp
    if not S.isValid(s) or isObstacle(O,sp):
        return 0

    if a==0:
        if s==sp:
            return 1
        else:
            return 0
    elif a==1:
        if (sy + 1) == spy and sx == spx:
            return (1-globprop) + globprop/4
        elif isAdjacent(s,sp):
            return globprop/4
        else:
            return 0
    elif a==2:
        if (sx + 1) == spx and sy == spy:
            return (1 - globprop) + globprop / 4
        elif isAdjacent(s, sp):
            return globprop / 4
        else:
            return 0
    elif a==3:
        if (sy - 1) == spy and sx == spx:
            return (1 - globprop) + globprop / 4
        elif isAdjacent(s, sp):
            return globprop / 4
        else:
            return 0
    elif a == 4:
        if (sx - 1) == spx and sy == spy:
            return (1 - globprop) + globprop / 4
        elif isAdjacent(s, sp):
            return globprop / 4
        else:
            return 0
    else:
        print('uh oh')

R = [[2,2,1],[2,0,10],[4,0,-100],[4,1,-100],[4,2,-100],[4,3,-100],[4,4,-100],[4,5,-100]]
O = [[1,1],[2,1],[1,3],[2,3]]
S = StateSpace(6,5)
print(isAdjacent([0,0],[1,0]))
print(transition_p(S,O,[1,1],4,[0,1]))
