colors = [
          ['G', 'R', 'R','G','G'],
          ['G', 'G', 'R','G','G'],
          ['G', 'G', 'R','R','G'],
          ['G', 'G', 'G','G','G']
         ]

measurements = ['R','R','R','R','R']
motions = [[0,0],[0,1],[1,0],[1,0],[0,1]]

pMove = 0.8
sensorRight = 0.7
pStay = 1-pMove
sensorWrong = 1-sensorRight

def move(p,step):
    q = [[0.0 for cell in range(len(p[0]))] for row in range(len(p))]

    for i in range(len(p)):
        for j in range(len(p[i])):
            q[i][j] = (p[(i-step[0])%len(p)][(j-step[1])%len(p[i])])*pMove + (p[i][j])*pStay
    return q




def sense(p,obs,color):
    q = [[0.0 for row in range(len(p[0]))] for col in range(len(p))] 
    s = 0.0
    
    for r in range(len(color)):
        for c in range(len(color[0])):
            hit = (color[r][c]==obs)
            q[r][c] = (p[r][c]*(sensorRight*hit + (1-sensorRight)*(1-hit)))
            s += q[r][c] 

    for i in range(len(q)):
        for j in range(len(q[i])):
            q[i][j] /= s 
    return q
        

def localize(colors,measurements,motions,sensorRight,pMove):
    priorValue = 1/(len(colors)*len(colors[0]))
    p = [[ priorValue for cell in row] for row in colors]
    for i in range(len(measurements)):
        #print(measurements[i], motions[i])
        p = move(p, motions[i])
        p = sense(p, measurements[i],colors)
        print('                             ......')
    return p

def show(p):
    rows = ['[' + ','.join(map(lambda x: '{0:.5f}'.format(x),r)) + ']' for r in p]
    print('[' + ',\n '.join(rows) + ']')    

p = localize(colors,measurements,motions,sensorRight,pMove)  
show(p)
