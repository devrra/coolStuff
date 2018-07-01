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
goal = [len(grid)-1, len(grid[0])-1]
cost = 1
G = 0
delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']
Open = []

def search(grid,init,goal,cost):
    # ----------------------------------------
    # insert code here
    # ----------------------------------------
    visited = grid[:][:] ## 0 - can be visited, 1- do not visit
    class cell:
        def __init__(self,pos,g):
            self.pos = pos
            self.g = g
            self.visit()
            if pos == goal:
                global G
                done = True
                G = self.g
                
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
    done = False
    
    #print(Open[0])
    
    while not done:
        #print(Open[0].g,Open[0].pos)
        try:
            Open[0].explore()
        except:
            break
    if G:    
        path = [G]+goal    
        return path
    else:
        return 'fail'
        
print(search(grid,init,goal,cost))
