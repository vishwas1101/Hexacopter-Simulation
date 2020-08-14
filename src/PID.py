#!/usr/bin/env python

import rospy
import math
import time
import numpy as np


def resetPID():
	return


def computeP(kp, err):
	return kp * err

def computeI(ki, I, err, dt, max, min):
	I = I + (err * dt)

	if I > max:
		I = max

	if I < min:
		I = min

	return I*ki

def computeD(kd, err, prev, dt):
	return kd * (err - prev) / dt

def PID(x, y, z, xVel, yVel, zVel, roll, pitch, yaw, f):

	# Set variables as global to prevent new assignment
	# P, I and D proportionality constants for position in the xyz axes
	global kpx, kix, kdx
	global kpy, kiy, kdy
	global kpz, kiz, kdz

	# P, I and D proportionality constants for velocity in the xyz axes	
	global kpvelx, kivelx, kdvelx
	global kpvely, kively, kdvely
	global kpvelz, kivelz, kdvelz

	# P, I and D proportionality constants for roll, pitch and yaw
	global kproll, kiroll, kdroll
	global kppitch, kipitch, kdpitch 
	global kpyaw, kiyaw, kdyaw

	# Errors for Parameters
	global prevErrorRoll, prevErrorPitch, prevErrorYaw
	global prevErrorx, prevErrory, prevErrorz
	global prevErrorVelx, prevErrorVely, prevErrorVelz

	# P, I and D for position in xyz axes
	global P_x, I_x, D_x
	global P_y, I_y, D_y
	global P_z, I_z, D_z

	# P, I and D for velocity in the xyz axes	
	global P_velx, I_velx, D_velx
	global P_vely, I_vely, D_vely
	global P_velz, I_velz, D_velz


	# P, I and D for roll, pitch and yaw
	global P_roll, I_roll, D_roll
	global P_pitch, I_pitch, D_pitch
	global P_yaw, I_yaw, D_yaw

	global setPointRoll, setPointPitch, setPointYaw 
	global setPointx, setPointz, setPointz

	global flag, sampleTime

	t = rospy.get_time()
	currTime = time.time()
	print(t)

	'''
	kproll = 20
	kiroll = 0
	kdroll = 89
	kppitch = kproll
	kipitch = kiroll
	kdpitch = kdroll
	'''

	# PID constants 
	kpyaw = 70
	kiyaw = 0
	kdyaw = 0

	sampleTime = 0

	kpx = 70
	kix = 0.0002
	kdx = 89
	kpy = 79
	kiy = 0.0002
	kdy = 89

	#tuned
	kpz = 1500
	kiz = 0
	kdz = 0
	kpvelx = 100
	kivelx = -0.0001
	kdvelx = 200
	kpvely = 200
	kively = -0.0001
	kdvely = 100

	kpvelz = 25
	kivelz = 0
	kdvelz = 0

	setPointYaw = 0
	errYaw = math.degrees(float(yaw)) - setPointYaw

	setPointx = 0
	setPointy = 0

	setPointz = 3.275

	errx = x - setPointx
	erry = y - setPointy
	errz = z - setPointz
	


	flag = 0

	if flag == 0: # What purpose does this serve? flag was set to 0 just 2 statements back
		prevTime = 0

		prevErrorRoll = 0
		prevErrorPitch = 0
		prevErrorYaw = 0

		prevErrorx = 0
		prevErrory = 0
		prevErrorz = 0

		prevErrorVelx = 0
		prevErrorVely = 0
		prevErrorVelz = 0

		P_roll = 0
		P_pitch = 0
		P_yaw = 0

		P_x = 0
		P_y = 0
		P_z = 0

		P_velx = 0
		P_vely = 0
		P_velz = 0

		I_roll = 0
		I_pitch = 0
		I_yaw = 0

		I_x = 0
		I_y = 0
		I_z = 0

		I_velx = 0
		I_vely = 0
		I_velz = 0

		D_roll = 0
		D_pitch = 0
		D_yaw = 0

		D_x = 0
		D_y = 0
		D_z = 0

		D_velx = 0
		D_vely = 0
		D_velz = 0

		flag = 1

	dt = currTime - prevTime

	if dt >= sampleTime:

		P_x = computeP(kpx, errx)
		P_y = computeP(kpy, erry)
		P_z = computeP(kpz, errz)

		I_x = computeI(kix, I_x, errx, dt, 600, -600)
		I_y = computeI(kiy, I_y, erry, dt, 600, -600)
		I_z = computeI(kiz, I_z, errz, dt, 600, -600)

		D_x = computeD(kdx, errx, prevErrorx, dt) 
		D_y = computeD(kdy, erry, prevErrory, dt) 
		D_z = computeD(kdy, errz, prevErrorz, dt) 

		P_yaw = computeP(kpyaw, errYaw)
		I_yaw = computeI(kiyaw, I_yaw, errYaw, dt, 600, -600)
		D_yaw = computeD(kdyaw, errYaw, prevErrorYaw, dt)

	#desVelx = -0.1 #P_x + I_x + D_x
	#desVely = -0.1 #P_y + I_y + D_y


	'''
	#Ignore this, it was for a temporary purpose to plan trajectory in probably the worst way possible!
	#Horizontal demo 1
	if(t<=27):
		desVelx = -0.1 #P_x + I_x + D_x
		desVely = -0.1 #P_y + I_y + D_y
	if(t>27 and t<=28.5):
		desVelx = -0.6
		desVely = -1.9
	if(t>28.5 and t<=30):
		desVelx = -0.6
		desVely = 3.5
	if(t>30 and t<=40):
		desVelx = -2.5
		desVely = 7
	if(t>40): 
		desVelx = +5
		desVely = +6
	'''

	
	#Horizontal demo 2 
	if(t<=34):
		desVelx = -0.5 #P_x + I_x + D_x
		desVely = 0.5 #P_y + I_y + D_y
	if(t>34 and t<=35):
		desVelx = -0.8
		desVely = 1.3
	if(t>35 and t<=36.5):
		desVelx = -0.8
		desVely = -7
	if(t>36.5 and t<=48):
		desVelx = -2.7
		desVely = -7
	if(t>48):
		desVelx = 0.0
		desVely = 0.0
	
	desVelz = P_z + I_z + D_z

	newYaw = P_yaw + I_yaw + D_yaw

	errVelx = xVel - desVelx
	errVely = yVel - desVely
	errVelz = zVel - desVelz

	if (dt >= sampleTime):

		P_velx = computeP(kpvelx, errVelx)
		P_vely = computeP(kpvely, errVely)
		P_velz = computeP(kpvelz, errVelz)

		I_velx = computeI(kivelx, I_velx, errVelx, dt, 600, -600)
		I_vely = computeI(kively, I_vely, errVely, dt, 600, -600)
		I_velz = computeI(kivelz, I_velz, errVelz, dt, 600, -600)

		D_velx = computeD(kdvelx, errVelx, prevErrorVelx, dt)
		D_vely = computeD(kdvely, errVely, prevErrorVely, dt)
		D_velz = computeD(kdvelz, errVelz, prevErrorVelz, dt)

	newAccx = P_velx + I_velx + D_velx
	newAccy = P_vely + I_vely + D_vely
	newThrottle = P_velz + I_velz + D_velz

	newRoll = -newAccy/9.8
	newPitch = newAccx/9.8

	errRoll = newRoll - roll
	errYaw = newYaw - yaw
	errPitch = newPitch - pitch

	prevTime = currTime

	prevErrorx = errx
	prevErrory = erry
	prevErrorz = errz

	prevErrorYaw = errYaw

	prevErrorVelx = errVelx
	prevErrorVely = errVely
	prevErrorVelz = errVelz

	esc_br = 1500 - desVelz + newPitch + 0.5*newRoll + newYaw
	esc_fr = 1500 - desVelz - newPitch + 0.5*newRoll + newYaw
	esc_fl = 1500 - desVelz - newPitch - 0.5*newRoll - newYaw
	esc_bl = 1500 - desVelz + newPitch - 0.5*newRoll - newYaw
	esc_r = 1500 - desVelz + newRoll - newYaw
	esc_l = 1500 - desVelz - newRoll + newYaw

	'''
	Ignore this...
	

	esc_br = 1500 - newRoll - newPitch + newYaw + newThrottle
	esc_bl = 1500 + newRoll - newPitch - newYaw + newThrottle
	esc_fl = 1500 + newRoll + newPitch - newYaw + newThrottle	
	esc_fr = 1500 - newRoll + newPitch + newYaw + newThrottle
	esc_r = 1500 - newRoll - newYaw + newThrottle
	esc_l = 1500 + newRoll + newYaw + newThrottle
	

	esc_br = 1500 + newPitch + newRoll + newYaw
	
	esc_br = 1500 + newRoll + newPitch + newYaw + newThrottle
	esc_bl = 1500 - newRoll + newPitch - newYaw + newThrottle
	esc_fl = 1500 - newRoll - newPitch + newYaw + newThrottle	
	esc_fr = 1500 + newRoll - newPitch - newYaw + newThrottle
	esc_r = 1500 - newRoll + newPitch - newYaw + newThrottle
	esc_l = 1500 + newRoll - newPitch + newYaw + newThrottle

	

	esc_br = newthrottle + newPitch + newRoll + newYaw
    esc_fr = newThrottle - newPitch + newRoll + newYaw
    esc_fl = newThrottle - newPitch - newRoll - newYaw
    esc_bl = newThrottle + newPitch - newRoll - newYaw
    esc_r = newThrottle + newRoll - newYaw  
    esc_l = newThrottle - newRoll + newYaw  

    '''  

	if(esc_br > 2000): esc_br = 2000
	if(esc_bl > 2000): esc_bl = 2000
	if(esc_fr > 2000): esc_fr = 2000
	if(esc_fl > 2000): esc_fl = 2000
	if(esc_r > 2000): esc_r = 2000
	if(esc_l > 2000): esc_l = 2000
		
	if(esc_br < 1100): esc_br = 1100
	if(esc_bl < 1100): esc_bl = 1100
	if(esc_fr < 1100): esc_fr = 1100
	if(esc_fl < 1100): esc_fl = 1100
	if(esc_r < 1100): esc_r = 1100
	if(esc_l < 1100): esc_l = 1100

	br_motor_vel = ((esc_br - 1500)/25) + 50
	bl_motor_vel = ((esc_bl - 1500)/25) + 50
	fr_motor_vel = ((esc_fr - 1500)/25) + 50
	fl_motor_vel = ((esc_fl - 1500)/25) + 50
	r_motor_vel = ((esc_r - 1500)/25) + 50
	l_motor_vel = ((esc_l - 1500)/25) + 50

	f.data = [fr_motor_vel, -fl_motor_vel, l_motor_vel, -bl_motor_vel, br_motor_vel, -r_motor_vel]

	return f, errRoll, errPitch, errYaw, errx, erry, errz


