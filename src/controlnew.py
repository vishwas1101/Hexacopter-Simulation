#!/usr/bin/env python

from PID import PID
import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray, Float32
from geometry_msgs.msg import Pose, Twist
from tf.transformations import euler_from_quaternion

def control_kwad(msg, args):
	
	global x, y, z, xvel, yvel, zvel, roll, pitch, yaw, err_roll, err_pitch, err_yaw
	
	#Assign the Float64MultiArray object to 'f' as we will have to send data of motor velocities to gazebo in this format
	f = Float64MultiArray()
	
	#quaternion to roll, pitch, yaw data
	ind = msg.name.index('Kwad')
	orientationObj = msg.pose[ind].orientation
	positionObj = msg.pose[ind].position
	velocityObj = msg.twist[ind].linear
	orientationList = [orientationObj.x, orientationObj.y, orientationObj.z, orientationObj.w]
	velocityList = [velocityObj.x, velocityObj.y, velocityObj.z]
	positionList = [positionObj.x, positionObj.y, positionObj.z]

	x = positionList[0]
	y = positionList[1]
	z = positionList[2]

	xvel = velocityList[0]
	yvel = velocityList[1]
	zvel = velocityList[2]

	(roll, pitch, yaw) = (euler_from_quaternion(orientationList))
	
	
	(fUpdated, err_roll, err_pitch, err_yaw, err_x, err_y, err_z) = PID(x, y, z, xvel, yvel, zvel, roll, pitch, yaw, f)


	args[0].publish(fUpdated)
	args[1].publish(err_roll)
	args[2].publish(err_pitch)
	args[3].publish(err_yaw)

#Initiate the node that will control the model
rospy.init_node("Control")

#initiate publishers that publish errors so that it can be plotted via rqt_plot to help in tuning
err_rollPub = rospy.Publisher('err_roll', Float32, queue_size=1)
err_pitchPub = rospy.Publisher('err_pitch', Float32, queue_size=1)
err_yawPub = rospy.Publisher('err_yaw', Float32, queue_size=1)
err_zPub = rospy.Publisher('err_z', Float32, queue_size=1)

#initialte publisher velPub that will publish the velocities of individual BLDC motors
velPub = rospy.Publisher('/Kwad/joint_motor_controller/command', Float64MultiArray, queue_size=6)

#Subscribe to /gazebo/model_states to obtain the pose in quaternion form

PoseSub = rospy.Subscriber('/gazebo/model_states',ModelStates,control_kwad,(velPub, err_rollPub, err_pitchPub, err_yawPub))

rospy.spin()
