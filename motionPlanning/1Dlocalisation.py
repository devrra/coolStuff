#Given the list motions=[1,1] which means the robot 
#moves right and then right again, compute the posterior 
#distribution if the robot first senses red, then moves 
#right one, then senses green, then moves right again, 
#starting with a uniform prior distribution.

'''
here the main thing is
                                     P(sensesGreen | greenCell) x P(greenCell)
  P(greenCell | sensesGreen) =  ------------------------------------------------------
                                  summation[ P(anyCell) x P(sensesGreen | anyCell) ]
                                  
  P( X<i,t> ) = summation[ P( X<j,t-1> ) . P( to X<i>|from X<j> )]

      X<i,t> = event of robot being at 'i'th cell at the 't'th step
      j is the postion of robot at 't-1'th step.

  
  sensing probabilities...
    pHit = P(sensesGreen | greenCell) = P(sensesRed | redCell) = 0.6
    pMiss = P(sensesGreen | redCell) = P(sensesRed | greenCell) = 0.2

  moving probablities...
    pExact = P( robot moves to the right cell ) = 0.8
    pUndershoot = P( robot moves to one cell before the right cell ) = 0.1
    pOvershoot = P( robot moves to one cell ahead the right cell ) = 0.1

  moving -- this leads the robot to lose some certainity of its position.
  sensing -- this leads the robot to gain some certainity of its position.
  
  all of localisation process is --
     -- to do the prior distribution of probablity of its position(the most uncertain state).
     -- sense - update
     -- move - update
     -- repeat the 2nd and 3rd step.

  memory requirement grows exponentially with the no. of dimesions(x,y,theta,phi, etc...)   
                                  
'''

p=[0.2, 0.2, 0.2, 0.2, 0.2] ## this is a Prior Distribution-Most Confused state-all cell has equal probability
world=['green', 'red', 'red', 'green', 'green']

measurements = ['red', 'red']
motions = [1,-1]

pHit = 0.6
pMiss = 0.2

pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1

def sense(p, Z): ## to update probaility distribution when the robot senses a cell color---P(anyCell | sensesAny), Any-color
    q=[]
    for i in range(len(p)):
        hit = (Z == world[i])       ## 0 or 1
        q.append(p[i] * (hit * pHit + (1-hit) * pMiss))
    s = sum(q)
    for i in range(len(q)):     ##NOrmalisation
        q[i] = q[i] / s
    return q

def move(p, U): ## to update probaility distribution when the robot moves.--- P( X<i,t> )
    '''
    U - robot's step size. 
    '''
    q = []
    for i in range(len(p)):
        s = pExact * p[(i-U) % len(p)]
        s = s + pOvershoot * p[(i-U-1) % len(p)]
        s = s + pUndershoot * p[(i-U+1) % len(p)]
        q.append(s)
    return q

if __name__ == '__main__':
    for i in range(2):
        p = sense(p, measurements[i])
        p = move(p, motions[i])

    print(p)     
