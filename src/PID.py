from math import * 
import time
import numpy as np

def PID(x,y,z, x_vel, y_vel, z_vel, roll, pitch, yaw, f):

	global kpx, kix, kdx
	global kpy, kiy, kdy
	global kpz, kiz, kdz

	global kproll, kiroll, kdroll
	global kppitch, kipitch, kdpitch 
	global kpyaw, kiyaw, kdyaw

	global prevErrorRoll, prevErrorPitch, prevErrorYaw
	global prevErrorx, prevErrory, prevErrorz

	global P_roll, P_pitch, P_yaw
	global I_roll, I_pitch, I_yaw
	global D_roll, D_pitch, D_yaw

	global P_x, P_y, P_z
	global I_x, I_y, I_z
	global D_x, D_y, D_z

	global setPointRoll, setPointPitch, setPointYaw 
	global setPointx, setPointz, setPointz

	global flag, sampleTime

	kproll = 70
	kiroll = 0
	kdroll = 89
	kppitch = kp_roll
	kipitch = ki_roll
	kdpitch = kd_roll
	kpyaw = 0.01
	kiyaw = 0
	kdyaw = 0

	flag = 0
	sampleTime = 0

	kpx = 1500
	kix = 0.0002
	kdx = 100
	kpy = 1500
	kiy = 0.0002
	kdy = 100
	kpz = 1500
	kiz = 0.0002
	kdz = 100

	
	setPointz = 3.275

	errz = z - setPointz

    if(errz <= 0.1)
		setPointx = 2
	 	setPointy = 0
	else:
		setPointx = 0
	 	setPointy = 0

	errx = x - setPointx
	erry = y - setPointy
	
	currTime = time.time()

	if flag == 0:
		prevTime = 0

		prevErrorRoll = 0
		prevErrorPitch = 0
		prevErrorYaw = 0

		prevErrorx = 0
		prevErrory = 0
		prevErrorz = 0

		P_roll = 0
		P_pitch = 0
		P_yaw = 0

		P_x = 0
		P_y = 0
		P_z = 0

		I_roll = 0
		I_pitch = 0
		I_yaw = 0

		I_y = 0
		I_x = 0
		I_z = 0

		D_roll = 0
		D_pitch = 0
		D_yaw = 0

		D_x = 0
		D_y = 0
		D_z = 0

		flag += 1

	dt = currTime - prevTime

	if(dt >= sampleTime):

		P_x = kpx * errx
		P_y = kpy * erry
		P_z = kpz * errz

