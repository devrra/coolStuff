from math import *
from copy import deepcopy
import matplotlib.pyplot as plt
import cv2
import numpy as np


grid = [
       [0,0,0,0,1,0,0,0,1,0,0,0,0,0,0],
       [0,0,0,0,1,0,0,0,1,0,0,0,0,0,0],
       [0,0,0,0,1,0,1,1,1,0,1,1,1,1,0],
       [0,0,0,0,0,0,1,0,0,0,1,0,1,0,0],
       [1,1,0,0,1,0,1,0,0,0,1,0,1,0,0],
       [0,0,0,0,1,0,1,0,0,0,0,0,1,0,0],
       [0,0,1,1,1,0,1,0,0,1,1,1,1,0,0],
       [0,0,0,0,1,0,1,0,0,0,0,1,0,0,0],
       [0,0,0,0,1,0,1,0,0,0,0,1,0,0,0],
       [0,0,0,0,1,0,1,0,0,0,0,1,1,1,0],
       [0,0,0,0,1,0,1,1,1,1,1,1,0,0,0],
       [0,0,0,0,1,0,1,0,0,0,0,1,0,0,0],
       [0,0,0,0,1,0,0,0,0,1,0,0,0,1,0],
       [0,0,0,0,1,0,0,0,0,0,0,0,0,1,0],
       [0,0,0,0,1,0,0,0,0,0,0,0,0,1,0]
       ]
init = [7,3,0]
goal = [2,9]


##grid = [
##        [0,1,0,0,0,0],
##        [0,1,0,0,0,0],
##        [0,1,0,0,1,0],
##        [0,0,0,1,0,0],
##        [0,0,0,0,1,0]
##        ]
##init = [4,0,0]
##goal = [3,4]

##grid = [
##        [0,0,1,0,0,0],
##        [0,0,1,0,1,0],
##        [0,0,1,0,1,0],
##        [0,0,0,0,1,0],
##        [0,0,1,0,1,0],
##        [0,0,1,0,0,0]
##        ]
##init = [3,0,3]
##goal = [3,5]

R = len(grid)
C = len(grid[0])
cost = [1,2,20]    ##[1,3,1] f,r,l  //final-initial

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

def generateHeuristic(init,R,C):
    heuristic = [[0 for c in range(C)] for r in range(R)]
    visited = [[0 for c in range(C)] for r in range(R)]
    r,c = init
    visited[r][c] = 1 
    h = 0
    Open = [[h]+init]
    while len(Open)!=0:
        h,r,c = Open.pop()
        for i in range(len(delta)): 
            dr,dc = delta[i]
            if r+dr in range(R) and c+dc in range(C) and visited[r+dr][c+dc]==0:
                heuristic[r+dr][c+dc] = h+1
                Open.append([h+1,r+dr,c+dc])  
                visited[r+dr][c+dc] = 1
        Open.sort()       
        Open.reverse() 
    return heuristic

def A_star(grid,init,goal,cost):
    heuristic = generateHeuristic(init[:2], len(grid), len(grid[0]))
    closed = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
    closed[init[0]][init[1]] = 1

    expand = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
    action = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]

    x,y,orient = init
    g = 0

    Open = [[g+heuristic[x][y], g, x, y,orient]]

    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand
    count = 0
    
    while not found and not resign:
        if len(Open) == 0:
            resign = True
            return "Fail"
        else:
            Open.sort()
            Open.reverse()
            Next = Open.pop()
            f,g,x,y,orient = Next
            expand[x][y] = count
            count += 1
            
            if x == goal[0] and y == goal[1]:
                found = True
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            ##comput action - i is newOrient
                            if abs(i-orient) == 3:
                                if orient == 3:    # left
                                    action = 2
                                else:
                                    action = 1
                            else:
                                action = (i-orient)%3
                            g2 = g + cost[action]
                            Open.append([heuristic[x2][y2]+g2, g2, x2, y2,i])
                            closed[x2][y2] = 1

    return expand

def generatePath(expand):
    #visited = grid[:][:]
    r,c = goal
    discreteRoute = [[r,c]]
    while [r,c]!=init[:2]:
        E = expand[goal[0]][goal[1]]+1
        I = -1
        for i in range(len(delta)):
            dr, dc = delta[i]
            if r+dr in range(R) and c+dc in range(C):# and visited[r+dr][c+dc]==0:
                if expand[r+dr][c+dc]<E and expand[r+dr][c+dc]!=-1:
                    E = expand[r+dr][c+dc]
                    I = i
        discreteRoute.append([r+delta[I][0],c+delta[I][1]])            
        r = r+delta[I][0]
        c = c+delta[I][1]
    discreteRoute.reverse()    
    return discreteRoute    

def improvise(path):     ## to introduce intermediate points in the path.
    detailedPath = [path[0]]
    for i in range(1,len(path)):
        x0,y0 = path[i-1]
        x1,y1 = path[i]
        dx,dy = (x1-x0)/4.0,(y1-y0)/4.0
        for j in range(1,4):
            detailedPath.append([x0+dx*j,y0+dy*j])
        detailedPath.append(path[i])         
    return detailedPath        

def smooth(path, weight_data = 0.5, weight_smooth = 0.42, tolerance = 0.000001):
    ## discrete->cont. Domain | returns set of real points. 
    newpath = deepcopy(path)
    change = tolerance
    while change>=tolerance:
        change = 0.0
        for i in range(1,len(path)-1):
            for j in range(2):
                aux = newpath[i][j]
                newpath[i][j] += weight_data*(path[i][j]-newpath[i][j]) 
                newpath[i][j] += weight_smooth*(newpath[i+1][j]+newpath[i-1][j]-2*newpath[i][j])
                change += abs(aux-newpath[i][j])
    return newpath

screenSize = 660
hor = len(grid[0])
ver = len(grid)
hor_halfCell = int(screenSize/(2*hor))
ver_halfCell = int(screenSize/(2*ver))

def Draw(path,grid):
    cv2.namedWindow("Map")
    Map = np.zeros((screenSize,screenSize,3), dtype = np.uint8)    
    cv2.circle(Map,(int((goal[1]+0.5)*ver_halfCell*2),int((goal[0]+0.5)*hor_halfCell*2)),7,(0,255,255),-1)
    cv2.circle(Map,(int((init[1]+0.5)*ver_halfCell*2),int((init[0]+0.5)*hor_halfCell*2)),7,(255,255,0),-1)
    for i in range(len(grid)):   ## drawing obstacles.
        for j in range(len(grid[0])):
            if grid[i][j] == 1:
                x0,y0 = int((j+1/2)*screenSize/hor - hor_halfCell),int((i+1/2)*screenSize/ver - ver_halfCell)
                x1,y1 = int((j+1/2)*screenSize/hor + hor_halfCell),int((i+1/2)*screenSize/ver + ver_halfCell)
                cv2.rectangle(Map,(x0,y0),(x1,y1),(0,0,255),-1)            
    for i in range(1,len(path)-1):
        x,y = int((path[i][0]+0.5)*screenSize/hor), int((path[i][1]+0.5)*screenSize/ver)              
        cv2.circle(Map,(y,x),3,(0,255,0),-1)   
    cv2.imshow("Map",Map)
        
if __name__ == '__main__':
    expand = A_star(grid, init, goal, cost)
    discreteRoute = generatePath(expand)
    detailedPath = improvise(discreteRoute)
    smoothPath = smooth(detailedPath)
    Draw(smoothPath,grid)
