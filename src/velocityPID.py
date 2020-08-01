
import math
import time
import numpy as np

class PID: 
	def __init__(self, kp, ki, kd):
		self.kp = kp
		self.ki = ki
		self.kd = kd

		self.target = 0
        self.output = 0
        self.error = 0

        self.resetPID()
        self.prevTime = time.time()


    def updatePID(self, target, state, time): 
    	self.target = target
        self.state = state
        self.time = time

        self.error = self.target - self.state
        dt = self.time - self.prevTime

        p = self.error
        self.intError += self.error * dt
        if (dt > 0):
            d = (self.error - self.prevError)  / dt
        else:
            d = 0

        if(self.intError > 600): i = 600
        elif(self.intError < -600): i = -600
        else: i = self.intError

        self.prevTime = self.time
        self.prevError = self.error

        output = self.kp * p + self.ki * i + self.kd * d

       	return output

	#set PID constants 
	def setKp(self, kp):
        self.kp = kp

    def setKi(self, ki):
        self.ki = ki

    def setKd(self, kd):
        self.kd = kd

    #reseting the PID	
	def resetPID(self): 
		self.target = 0.0
        self.error = 0.0
        self.state = 0.0
        self.intError = 0.0
        self.prevError = 0.0
        self.output = 0.0
