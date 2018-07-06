forward = [[-1,0],[0,-1],[1,0],[0,1]]
forward_name = ['up','left','down','right']
action = [-1,0,1]
action_name = ['R','#','L']

grid = [[1,1,1,0,0,0],
        [1,1,1,0,1,0],
        [0,0,0,0,0,0],
        [1,1,1,0,1,1],
        [1,1,1,0,1,1]]
init = [4,3,0]
goal = [2,0]
cost = [2,1,20]

def optimum_policy2D(grid, init, goal, cost):
    value = [[999*i for i in row] for row in grid]
    #print(value)
    r,c,orientation = init
    Routes = []
    Routes.append([0,[init[:2]],orientation])

    while [r,c]!=goal:
        #print([r,c])
        presentRoute = Routes[0]
        for i in range(len(action)):
            #print('----i = ',i,'-----')
            dr, dc = forward[(presentRoute[2]+action[i])%4]
            #print('            ',(dr,dc),'---',action_name[i])
            if r+dr in range(len(grid)) and c+dc in range(len(grid[0])) and value[r+dr][c+dc]!=999:
                thisRoute = presentRoute[:]
                thisRoute[1].append([r+dr,c+dc])
                thisRoute[0] += cost[i]
                thisRoute[2] = (thisRoute[2]+action[i])%4
                ##print('            thisRoute\'s cost : ',thisRoute[0])
                ##print('            thisRoute\'s orientation : ',forward_name[thisRoute[2]])
                ##print('            thisRoute\'s last cell : ',thisRoute[1][-1])
                Routes.append(thisRoute[:])
                del thisRoute
                ##print('            number of Routes : ',len(Routes))
                ##print('\n next FOR Loop iteration \n-------------------------------------')
        Routes = Routes[1:]
        Routes.sort()
        r,c = Routes[0][1][-1]
        ##print('\n next WHILE Loop iteration \n========================================================..')
    print(Routes[0])

optimum_policy2D(grid, init, goal, cost)    
    
