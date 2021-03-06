# ----------
# User Instructions:
# 
# Define a function, search() that returns a list
# in the form of [optimal path length, row, col]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-4]
cost = 1
G = 0
E = 0
done = False
delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']
Open = []

def search(grid,init,goal,cost):
    visited = grid[:][:] ## 0 - can be visited, 1- do not visit
    expand = [[-1 for cell in row] for row in grid]
    class cell:
        def __init__(self,pos,g):
            self.pos = pos
            self.g = g
            self.visit()
            r,c = pos
            global E
            expand[r][c] = E
            E += 1
            if pos == goal:
                global G
                global done
                done = True
                G = self.g
                #print('done   :   ',done)
                
        def visit(self):
            i,j = self.pos
            visited[i][j]=1
            
        def explore(self):
            global Open
            r,c = self.pos
            for n in range(len(delta)):
                dr,dc = delta[n]
                try:
                    if visited[r+dr][c+dc] != 1 and (r+dr) in range(len(grid)) and (c+dc) in range(len(grid[0])):
                        Open.append(cell([r+dr,c+dc],self.g+1))
                except:
                    pass
            Open = Open[1:]
    
    global Open
    Open = [cell(init,G)]
    #done = False
    
    #print(Open[0])
    
    while not done:
        #print(Open[0].g,Open[0].pos)
        try:
            Open[0].explore()
        except:
            break
    #print(expand)    

    direction = [[' ' for cell in row] for row in grid]
    r, c = 0,0
    while [r,c]!=goal:
        maxi = -1
        #print(r,c)
        for i in range(len(delta)):
            dr,dc = delta[i]
            if (r+dr) in range(len(grid)) and (c+dc) in range(len(grid[0])):
                if expand[r+dr][c+dc]>maxi:
                    maxi = expand[r+dr][c+dc]
                    D = i
        direction[r][c] = delta_name[D]
        r += delta[D][0]
        c += delta[D][1]
    direction[r][c] = '*'        
    for i in range(len(direction)):
        print(direction[i])
    if G:    
        path = [G]+goal    
        return path
    else:
        return 'fail'
        
path = print(search(grid,init,goal,cost))
print(path)
