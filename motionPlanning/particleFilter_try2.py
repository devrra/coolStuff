## IT TAKES NOISE INTO ACCOUNT.
from math import *
import random

landMarks = [[0.0,0.0],[0.0,100.0],[100.0,100.0],[100.0,0.0]]
worldSize = 100.0
maxSteer = pi/4

class robot:
    def __init__(self):
        self.x = random.random()*worldSize
        self.y = random.random()*worldSize
        self.orientation = random.random()*2*pi
        self.length = 20.0
        self.bearingNoise = 0.0
        self.steeringNoise = 0.0
        self.forwardNoise = 0.0
        self.senseNoise = 0.0

    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s ]'%(str(self.x),str(self.y),str(self.orientation))

    def move(self,motion):
        a, d = motion
        a2 = random.gauss(a,self.steeringNoise)
        d2 = random.gauss(d,self.forwardNoise)

        res = robot()
        res.length = self.length
        res.bearingNoise = self.bearingNoise
        res.steeringNoise = self.steeringNoise
        res.forwardNoise = self.forwardNoise
        res.senseNoise = self.senseNoise

        b = d2/res.length*tan(a2)
        if abs(b)<0.001:
            res.x = self.x+d2*cos(self.orientation+b)
            res.y = self.y+d2*sin(self.orientation+b)
            res.orientation = (self.orientation+b)%(2*pi)
        else:
            R = d2/b
            cx = self.x-sin(self.orientation)*R
            cy = self.y+cos(self.orientation)*R
            res.orientation = (b+self.orientation)%(2*pi)
            res.x = cx+sin(b+self.orientation)*R
            res.y = cy-cos(b+self.orientation)*R
        return res    

    def sense(self, addNoise=1):
        Z = []
        for i in range(len(landMarks)):
            bearing = atan2((landMarks[i][1]-self.y),(landMarks[i][0]-self.x))
            if addNoise:
                bearing += random.gauss(0,self.bearingNoise)
            Z.append(bearing)
        return Z    
                

    def setNoise(self,bearingNoise,steeringNoise,distanceNoise):
        self.bearingNoise = float(bearingNoise)
        self.steeringNoise = float(steeringNoise)
        self.distanceNoise = float(distanceNoise)

    def set(self,x,y,newOrientation):
        self.x = x
        self.y = y
        self.newOrientation = newOrientation

    def Gaussian(self,x,mu,sigma):
        return (1/sqrt(2*pi*sigma**2))*exp((-(x-mu)**2)/(2*sigma**2))

    def measurementProb(self,measurement):
        prob = 1.0
        for i in range(len(landMarks)):
            bearing = atan2((landMarks[i][1]-self.y),(landMarks[i][0]-self.x))
            prob *= self.Gaussian(measurement[i],bearing,self.bearingNoise)
        return prob
