#!/usr/bin/env python

# The Keyboard Controller Node for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials

# This controller extends the base DroneVideoDisplay class, adding a keypress handler to enable keyboard control of the drone

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from SAR_drone_controller import BasicDroneController
from drone_video_display import DroneVideoDisplay

# Finally the GUI libraries
from PySide import QtCore, QtGui


# Here we define the keyboard map for our controller (note that python has no enums, so we use a class)
class KeyMapping(object):
	PitchForward     = QtCore.Qt.Key.Key_I
	PitchBackward    = QtCore.Qt.Key.Key_K
	RollLeft         = QtCore.Qt.Key.Key_J
	RollRight        = QtCore.Qt.Key.Key_L
	YawLeft          = QtCore.Qt.Key.Key_U
	YawRight         = QtCore.Qt.Key.Key_O
	IncreaseAltitude = QtCore.Qt.Key.Key_W
	DecreaseAltitude = QtCore.Qt.Key.Key_E
	Takeoff          = QtCore.Qt.Key.Key_S
	Land             = QtCore.Qt.Key.Key_D
	Emergency        = QtCore.Qt.Key.Key_Space
# added for SAR
	mode_1 			 = QtCore.Qt.Key.Key_1 # liftoff
	mode_2 			 = QtCore.Qt.Key.Key_2 # land
	mode_3 			 = QtCore.Qt.Key.Key_3 # pick/load
	mode_4 			 = QtCore.Qt.Key.Key_4 # origin to tb
	mode_5 			 = QtCore.Qt.Key.Key_5 # tb to origin
	mode_6 			 = QtCore.Qt.Key.Key_6 # spin
	mode_7 			 = QtCore.Qt.Key.Key_7 # spin and fly


# Our controller definition, note that we extend the DroneVideoDisplay class
class KeyboardController(DroneVideoDisplay):
	def __init__(self):
		super(KeyboardController,self).__init__()
		
		self.pitch = 0
		self.roll = 0
		self.yaw_velocity = 0 
		self.z_velocity = 0
		#self.keypress = 1

# We add a keyboard handler to the DroneVideoDisplay to react to keypresses
	def keyPressEvent(self, event):
		key = event.key()
		# If we have constructed the drone controller and the key is not generated from an auto-repeating key
		if controller is not None and not event.isAutoRepeat():
			# manual override
			#self.keypress = 1
			# Handle the important cases first!
			if key == KeyMapping.Emergency:
				controller.SendEmergency()
			elif key == KeyMapping.Takeoff or key == KeyMapping.mode_1:
				controller.SendTakeoff()
			elif key == KeyMapping.Land or key == KeyMapping.mode_2:
				controller.SendLand()
			else: 
				# Now we handle moving MANUALLY, notice that this section is the opposite (+=) of the keyrelease section
				if key == KeyMapping.YawLeft:
					self.yaw_velocity += 1
				elif key == KeyMapping.YawRight:
					self.yaw_velocity += -1

				elif key == KeyMapping.PitchForward:
					self.pitch += 1
				elif key == KeyMapping.PitchBackward:
					self.pitch += -1

				elif key == KeyMapping.RollLeft:
					self.roll += 1
				elif key == KeyMapping.RollRight:
					self.roll += -1

				elif key == KeyMapping.IncreaseAltitude:
					self.z_velocity += 1
				elif key == KeyMapping.DecreaseAltitude:
					self.z_velocity += -1
			
				if key == KeyMapping.mode_1:
					pub_ctrl_mode.publish(1)
				
				if key == KeyMapping.mode_2:
					pub_ctrl_mode.publish(2)
				
				if key == KeyMapping.mode_3:
					pub_ctrl_mode.publish(3)
				
				if key == KeyMapping.mode_4:
					pub_ctrl_mode.publish(4)
				
				if key == KeyMapping.mode_5:
					pub_ctrl_mode.publish(5)

				if key == KeyMapping.mode_6:
					pub_ctrl_mode.publish(6)

				if key == KeyMapping.mode_7:
					pub_ctrl_mode.publish(7)


			# finally we set the command to be sent. The controller handles sending this at regular intervals
			controller.SetKeyboardCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)


	def keyReleaseEvent(self,event):
		key = event.key()

		# If we have constructed the drone controller and the key is not generated from an auto-repeating key
		if controller is not None and not event.isAutoRepeat():
			# Note that we don't handle the release of emergency/takeoff/landing keys here, there is no need.
			# Now we handle moving, notice that this section is the opposite (-=) of the keypress section
			
			# not manual override
			#self.keypress = 0

			if key == KeyMapping.YawLeft:
				self.yaw_velocity -= 1
			elif key == KeyMapping.YawRight:
				self.yaw_velocity -= -1

			elif key == KeyMapping.PitchForward:
				self.pitch -= 1
			elif key == KeyMapping.PitchBackward:
				self.pitch -= -1

			elif key == KeyMapping.RollLeft:
				self.roll -= 1
			elif key == KeyMapping.RollRight:
				self.roll -= -1

			elif key == KeyMapping.IncreaseAltitude:
				self.z_velocity -= 1
			elif key == KeyMapping.DecreaseAltitude:
				self.z_velocity -= -1

			# finally we set the command to be sent. The controller handles sending this at regular intervals
			controller.SetKeyboardCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
	

# Setup the application
if __name__=='__main__':
	print("SAR_drone_ctrl_node started")
	import sys
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('SAR_drone_ctrl_node')

	# publiseher for ctrl mode
	pub_ctrl_mode = rospy.Publisher('control_mode', Int32, queue_size=10)
	
	# Now we construct our Qt Application and associated controllers and windows
	app = QtGui.QApplication(sys.argv)
	controller = BasicDroneController()
	display = KeyboardController()
	display.show()

	# executes the QT application
	status = app.exec_()

	# and only progresses to here once the application has been shutdown
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)