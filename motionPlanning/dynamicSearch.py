grid = [[0,1,0,0,0,0],
        [0,1,0,0,0,0],
        [0,0,0,0,0,0],
        [0,1,0,0,0,0],
        [0,0,0,0,1,0]]
goal = [len(grid)-1,len(grid[0])-1]
cost = 1
delta = [[-1,0],
         [0,-1],
         [1,0],
         [0,1]]
delta_name = ['^','<','v','>']

def compute(grid,goal,cost):
    R = len(grid)
    C = len(grid[0])
    D = len(delta)
    value = [[99 for c in range(C)] for r in range(R)]
    direction = [['o' for c in range(C)] for r in range(R)]
    visited = grid[:][:]
    r,c = goal
    direction[r][c] = '*'
    visited[r][c] = 1
    value[r][c] = 0
    step = 0
    Open = [[step,r,c]]
    while len(Open)!=0:
        Open.sort()
        Open.reverse()
        Next = Open.pop()
        step, r, c = Next
        for i in range(D):
            dr, dc = delta[i]
            if (r+dr) in range(R) and (c+dc) in range(C):
                if visited[r+dr][c+dc] != 1:
                    value[r+dr][c+dc] = step+cost
                    visited[r+dr][c+dc] = cost
                    Open.append([step+cost,r+dr,c+dc])

    for r in range(R):
        for c in range(C):
            for i in range(D):
                dr, dc = delta[i]
                if (r+dr) in range(R) and (c+dc) in range(C):
                    if value[r][c]-value[r+dr][c+dc]==1:
                       direction[r][c]=delta_name[i]
                       break
                       
    return direction

result = compute(grid,goal,cost)
for row in result:
    for cell in row:
        print(cell,end='  ')
    print('')    
                    
