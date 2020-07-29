import math
import time
import numpy as np

def PID(x,y,z, xVel, yVel, zVel, roll, pitch, yaw, f):

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

	'''
	kproll = 20
	kiroll = 0
	kdroll = 89
	kppitch = kproll
	kipitch = kiroll
	kdpitch = kdroll
	'''

	# PID constants 
	kpyaw = 500
	kiyaw = 0.02 
	kdyaw = 50

	flag = 0
	sampleTime = 0

	kpx = 3000
	kix = 0.0002
	kdx = 3000
	kpy = 3000
	kiy = 0.0008
	kdy = 2000
	kpz = 5000
	kiz = 0.0002
	kdz = 2000

	kpvelx = 5
	kivelx = 0
	kdvelx = -5
	kpvely = 5
	kively = 0
	kdvely = -5
	kpvelz = 5
	kivelz = 0
	kdvelz = -5

	setPointYaw = 0
	errYaw = math.degrees(float(yaw)) - setPointYaw

	setPointx = 0
	setPointy = 0
	setPointz = 3.275

	errx = x - setPointx
	erry = y - setPointy
	errz = z - setPointz
	
	currTime = time.time()

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

		flag += 1

	dt = currTime - prevTime

	if(dt >= sampleTime):

		P_x = kpx * errx
		P_y = kpy * erry
		P_z = kpz * errz

		P_yaw = kpyaw * errYaw

		I_x += errx * dt
		I_y += erry * dt
		I_z += errz * dt

		I_yaw += errYaw * dt

		if(I_x > 600): I_x = 600
		if(I_y > 600): I_y = 600
		if(I_z > 600): I_z = 600

		if(I_x < -600): I_x = -600
		if(I_y < -600): I_y = -600
		if(I_z < -600): I_z = -600

		if(I_yaw > 600): I_yaw = 600
		if(I_yaw < -600): I_yaw = -600

		D_x = (errx - prevErrorx)/dt 
		D_y = (erry - prevErrory)/dt 
		D_z = (errz - prevErrorz)/dt 

		D_yaw = (errYaw - prevErrorYaw)/dt

	desVelx = P_x + kix * I_x + kdx * D_x
	desVely = P_y + kiy * I_y + kdy * D_y
	desVelz = P_z + kiz * I_z + kdz * D_z

	newYaw = P_yaw + kiyaw * I_yaw + D_yaw * kdyaw

	errVelx = xVel - desVelx
	errVely = yVel - desVely
	errVelz = zVel - desVelz

	if (dt >= sampleTime):

		P_velx = kpvelx * errVelx
		P_vely = kpvely * errVely
		P_velz = kpvelz * errVelz

		I_velx += errVelx * dt
		I_vely += errVely * dt
		I_velz += errVelz * dt

		if(I_velx > 600): I_velx = 600
		if(I_vely > 600): I_vely = 600
		if(I_velz > 600): I_velz = 600

		if(I_velx < -600): I_velx = -600
		if(I_vely < -600): I_vely = -600
		if(I_velz < -600): I_velz = -600

		D_velx = (errVelx - prevErrorVelx)/dt
		D_vely = (errVely - prevErrorVely)/dt
		D_velz = (errVelz - prevErrorVelz)/dt

	newRoll = P_velx + kivelx * I_velx + kdvelx * D_velx
	newPitch = P_vely + kively * I_vely + kdvely * D_vely
	newThrottle = P_velz + kivelz * I_velz + kdvelz * D_velz

	prevTime = currTime

	prevErrorx = errx
	prevErrory = erry
	prevErrorz = errz

	prevErrorYaw = errYaw

	prevErrorVelx = errVelx
	prevErrorVely = errVely
	prevErrorVelz = errVelz


	esc_br = newThrottle + newPitch + newRoll + newYaw
	esc_fr = newThrottle - newPitch + newRoll + newYaw
	esc_fl = newThrottle - newPitch - newRoll - newYaw
	esc_bl = newThrottle + newPitch - newRoll - newYaw
	esc_r = newThrottle + newRoll - newYaw
	esc_l = newThrottle - newRoll + newYaw

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

	return f, newRoll, newPitch, newYaw


