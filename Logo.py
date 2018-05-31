import cv2
import numpy as np

c1 = (320, 140)
c2 = (180, 380)
c3 = (460, 380)

color = {'R':(0,0,255),'G':(0,255,0),'B':(255,0,0)}

def nothing(x):
    pass

def setColor(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        r = cv2.getTrackbarPos('R', 'Logo')
        g = cv2.getTrackbarPos('G', 'Logo')
        b = cv2.getTrackbarPos('B', 'Logo')
        if red[x,y]==255:
            color['R']=[b,g,r]
        elif green[x,y]==255:
            color['G']=[b,g,r]
        elif blue[x,y]==255:
            color['B']=[b,g,r]
        else:
            pass
            
img = np.zeros((640, 640, 3), dtype = np.uint8)
cv2.namedWindow('Logo')

red = img[:,:,2]
green = img[:,:,1]
blue = img[:,:,0]

cv2.createTrackbar('R', 'Logo', 0, 255, nothing)
cv2.createTrackbar('G', 'Logo', 0, 255, nothing)
cv2.createTrackbar('B', 'Logo', 0, 255, nothing)


def Draw():
    cv2.circle(img,c1,120,color['R'],-1)     ##color['R']
    cv2.circle(img,c2,120,color['G'],-1)     ##color['G']
    cv2.circle(img,c3,120,color['B'],-1)     ##color['B']

    cv2.circle(img,c1,50,(0,0,0),-1)
    cv2.circle(img,c2,50,(0,0,0),-1)
    cv2.circle(img,c3,50,(0,0,0),-1)

    cv2.ellipse(img, c2, (120, 120), 0, -60, 0, 0, -1)
    cv2.ellipse(img, c1, (120, 120), 60, 60, 0, 0, -1)
    cv2.ellipse(img, c3, (120, 120), 0, -120, -60, 0, -1)
Draw()
cv2.setMouseCallback('Logo', setColor)

cv2.imshow('Logo', blue)

while True:
    Draw()
    cv2.imshow('Logo', img)

    k = cv2.waitKey(1)&0xFF
    if k == 27:
        break
#cv2.destroyAllWindows()
cv2.imshow('Logo', red)
