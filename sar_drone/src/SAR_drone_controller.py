#!/usr/bin/env python

# A basic drone controller class for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This class implements basic control functionality which we will be using in future tutorials.
# It can command takeoff/landing/emergency as well as drone movement
# It also tracks the drone state based on navdata feedback

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy
from std_msgs.msg import String
# numpy imports - basic math and matrix manipulation
import numpy as np


# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Int32
from std_msgs.msg import Empty      	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from geometry_msgs.msg import Pose2D
from optic_flow_example.msg import OpticFlowMsg

# An enumeration of Drone Statuses
from drone_status import DroneStatus



# Some Constants
COMMAND_PERIOD = 100 #ms 

class BasicDroneController(object):
	def __init__(self):
		# Holds the current drone status
		self.status = -1
		self.OverrideCounter = 10
		self.takeoff_counter = 40
		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 
		self.subLocalization = rospy.Subscriber('/drone_pose_est',Pose2D,self.RecieveLocalization)
		self.subOptFlow = rospy.Subscriber('/optic_flow',OpticFlowMsg,self.RecieveOpticalFlow)
		self.subRefPose = rospy.Subscriber('/drone_ref_pose',Pose2D,self.RecieveRefPose)

		# subscribe from action server
		self.subTakeOff = rospy.Subscriber('drone_takeoff', Empty, self.drone_takeoff_callback)
		self.subLand = rospy.Subscriber('drone_land', Empty, self.drone_land_callback)

		# Allow the controller to publish to the /ardrone/takeoff, land and reset topics
		self.pubLand    = rospy.Publisher('/ardrone/land',Empty, queue_size=1)
		self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty, queue_size=1)
		self.pubReset   = rospy.Publisher('/ardrone/reset',Empty, queue_size=1)		

		# Allow the controller to publish to the /cmd_vel topic and thus control the drone
		self.pubCommand = rospy.Publisher('/cmd_vel',Twist, queue_size=10)

		# Setup regular publishing of control packets
		self.command = Twist()
		self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand)

		# Added vars 
		self.batteryPercent = 100
		self.theta = 0 	
		self.x = 0	
		self.y = 0 

		self.Kp_x = 0.45
		self.Kp_y = 0.45
		self.Kp_theta = 2.0

		self.Kd_x = 1.1
		self.Kd_y = 1.1

		self.K_lean_x = 0 #0.02
		self.K_lean_y = 0 #0.02

		self.Ki_x = 0.00
		self.Ki_y = 0.00

		self.last_x = 0
		self.haslocCounter = 10	

		self.pitch = 0 # for d-control 
		self.roll = 0

		self.vx = 0
		self.vy = 0

		self.Ix = 0
		self.Iy = 0

		self.ref_pose = Pose2D()
		self.ref_pose.x = 0
		self.ref_pose.y = 0
		self.ref_pose.theta = 0

		self.Kp_alt = 1.0
		self.ref_alt = 1.5 # m
		self.alt = 0.0

		# Land the drone if we are shutting down
		rospy.on_shutdown(self.SendLand)

	def RecieveLocalization(self,dronepose):
#		print "in RecieveLocalization"
#		print dronepose.x
		self.x = dronepose.x		# fwd pos (m)
		self.y = dronepose.y 		# left pos (m)
		self.theta = dronepose.theta 	# orientation -pi to pi
		
# 		if(self.x == self.last_x):
#			self.haslocCounter = self.haslocCounter +1
#		else:
#			self.haslocCounter = 0
#
#		self.last_x = self.x

	def ReceiveNavdata(self,navdata):
		# Although there is a lot of data in this packet, we're only interested in the state at the moment	
		self.status = navdata.state
		self.batteryPercent = navdata.batteryPercent 
		self.roll = -navdata.rotX # left/right tilt in degrees (rotation about the X axis)
		self.pitch = navdata.rotY # forward/backward tilt in degrees (rotation about the Y axis)
		self.alt = navdata.altd/1000.0
 		self.vx = navdata.vx/1000.0
 		self.vy = navdata.vy/1000.0

	def RecieveOpticalFlow(self,optflow):
		pass
		#self.vx = np.median(optflow.vy, axis=0)
		#self.vy = np.median(optflow.vx, axis=0)

	def RecieveVelEst(self,drone_vel_est):
		pass
		#self.vx = drone_vel_est.linear.x
		#self.vy = drone_vel_est.linear.y

	def RecieveRefPose(self,refpose):
		self.ref_pose = refpose

	def drone_takeoff_callback(self,data):
		self.takeoff_counter = 0
		self.SendTakeoff()

	def drone_land_callback(self,data):
		self.SendLand()


	def SendTakeoff(self):
		# Send a takeoff message to the ardrone driver
		# Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
		self.takeoff_counter = 0
		if(self.status == DroneStatus.Landed):
			print "sent takeoff mgs"
			self.pubTakeoff.publish(Empty())
			
	def SendLand(self):
		# Send a landing message to the ardrone driver
		# Note we send this in all states, landing can do no harm
		self.pubLand.publish(Empty())

	def SendEmergency(self):
		# Send an emergency (or reset) message to the ardrone driver
		self.pubReset.publish(Empty())

	def SetKeyboardCommand(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0):
		
		self.OverrideCounter = 0
		# Called by the main program to set the current command
		
		self.command.linear.x  = pitch
		self.command.linear.y  = roll
		self.command.linear.z  = z_velocity
		self.command.angular.z = yaw_velocity

		# zero integral when keyboard
		self.Ix = 0
		self.Ix = 0

	def SetControllerCommand(self):
		print("controller running")
				
		# calculate control errors 
		e_theta = self.ref_pose.theta - self.theta 
		e_x = self.ref_pose.x - self.x
		e_y = self.ref_pose.y - self.y
		e_alt = 0 #self.ref_alt - self.alt

		# print control errors
		print("control errors:")
		print("e_alt:           %f" % e_alt)
		print("e_theta:         %f" % e_theta)
		print("e_x:             %f" % e_x)
		print("e_y:             %f" % e_y)
		print("v_x:             %f" % self.vx )
		print("v_y:             %f" % self.vy )
    	
		# print control actions
		print "contol actions"
		print("self.command.linear.x: %f" % self.command.linear.x)
		print("self.command.linear.y: %f" % self.command.linear.y)
		print("self.command.linear.z: %f" % self.command.linear.z)
		print("self.command.angular.z: %f" % self.command.angular.z)


		if(abs(self.Ix) < 2.0 or e_x*self.Ix < 0):
			self.Ix = self.Ix + e_x
		if(abs(self.Iy) < 2.0 or e_x*self.Ix < 0):
			self.Iy = self.Iy + e_y

		# print integral part
		#print "integral part"
		#print("Ix: %f" % self.Ix)
		#print("Iy: %f" % self.Iy)

		# print opt flox
		#print "opt flow"
		#print("vx: %f" % self.vx)
		#print("vy: %f" % self.vy)		

		print"\n REFS "
		print("x_ref = %f" %self.ref_pose.x)
		print("y_ref = %f" %self.ref_pose.y)
		print("theta_ref = %f" %self.ref_pose.theta)


		# handle no tag case
		if (self.haslocCounter <= 10):
			print("localization ok")
		else:
			print("localization not ok")
			self.command.linear.z  = 0
			self.command.angular.z = 0
			self.command.linear.x  = 0
			self.command.linear.y  = 0
    	
    	# calculate control
		vX = self.vx*np.cos(self.theta) - self.vy*np.sin(self.theta)
		vY = self.vx*np.sin(self.theta) + self.vy*np.cos(self.theta)

		self.command.linear.z = self.Kp_alt*e_alt
		self.command.angular.z = self.Kp_theta*e_theta

		#if e_theta < 0.3:
		cmd_X = self.Kp_y * e_x - self.Kd_x*vX - self.K_lean_x*self.pitch 
		cmd_Y = self.Kp_y * e_y - self.Kd_y*vY - self.K_lean_y*self.roll

		self.command.linear.x = cmd_X 
		self.command.linear.y = cmd_Y
		
		# handle nonzero yaw
		self.command.linear.x = cmd_X*np.cos(self.theta) + cmd_Y*np.sin(self.theta)
		self.command.linear.y = -cmd_X*np.sin(self.theta) + cmd_Y*np.cos(self.theta) 

		#else:
		#self.command.linear.x  = 0
		#self.command.linear.y  = 0	
		#if e_x > 1.0:
		#	self.command.linear.x 

		#if self.takeoff_counter % 5 == 0:
		#	self.hover = not self.hover
		eps = 0.05
		eps_theta = 0.1
		eps_alt = 0.3
		if (abs(e_x) < eps and abs(e_y) < eps and abs(e_theta) < eps_theta and abs(e_alt) < eps_alt):
		#if self.hover:
			print"hovering"
			self.command.linear.x = 0 
			self.command.linear.y = 0
			self.command.linear.z = 0
			self.command.angular.x = 0
			self.command.angular.y = 0
			self.command.angular.z = 0


	def SendCommand(self,event):
		print ""
		print("In send command")
		print "batteryPercent = %d" % self.batteryPercent

		self.takeoff_counter +=1
		# handle manual override
		if (self.OverrideCounter <= 20):
			print("manual override or takeoff wait")
			self.OverrideCounter += 1
		elif (self.takeoff_counter <= 50):
			print"TAKEOFF - controller disabled"
			self.takeoff_counter += 1
		else:
			print("automatic mode engaged")
			self.SetControllerCommand()

		# The previously set command is then sent out periodically if the drone is flying
		if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
			self.pubCommand.publish(self.command)



