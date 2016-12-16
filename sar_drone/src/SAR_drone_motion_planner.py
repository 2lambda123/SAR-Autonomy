#!/usr/bin/env python
#### SAR_drone_motion planner_node.py

import rospy
import roslib; roslib.load_manifest('sar_drone')
import tf
import numpy as np
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D 

# modes:
# 1 - liftoff
# 2 - land
# 3 - pick / load
# 4 - fly from crate (origin) area to load area
# 5 - fly from load area to crate area (origin)


def generate_ref():
	global ref_x
	ref_x = 0
	global ref_y
	ref_y = 0
	global ref_theta
	ref_theta = 0
	
	global mode
	mode = 0
	
	global count_6
	count_6 = 0

	global count_7
	count_7 = 0

	global count_4
	count_4 = 0

	print"\n Motion planner initiated "	
	dt = 1.0/20.0
	dist = 2.0 # distance in x between crate area and load area
	dist2 = 1.0
	v_ref = 0.2
	w_ref = 0.3 # rad/sec
	ref_pose = Pose2D()
	ref_pose.x = 0
	ref_pose.y = 0
	ref_pose.theta = 0

	while not rospy.is_shutdown():

		if mode == 1:
			# liftoff
			pass
		if mode == 2:
			# land
			pass
		if mode == 3:
			# pick/load
			pass
		if mode == 4:
			# fly from crate (origin) area to load area
			if ref_x <= dist:
				ref_x  += v_ref*dt
#			elif ref_x >= dist and count_4 <= 60:
#				ref_theta = -np.pi/2
#				count_4 += 1
#			elif ref_x >= dist and ref_y <= dist2 and count_4 >= 60:
#				ref_y = v_ref*dt
			else:
				ref_theta = -np.pi/2

		if mode == 5:
			# fly from load area to crate area (origin)
			if ref_x >= 0:
				ref_x  -= v_ref*dt
#			elif ref_y <= 0 and ref_x >= 0:
#				ref_x -= v_ref*dt
#				ref_theta = 0
				ref_theta = 0
		
		if mode == 6:
			# spin
#			ref_theta = 0.9
			if count_6 <= 40:
				ref_theta = 0
			elif count_6 > 40 and count_6 <= 70:
				ref_theta = 0.5
			elif count_6 > 70 and count_6 <= 130:
				ref_theta = -0.5
			else:
				ref_theta = 0

			
			
			count_6 += 1

		if mode == 7:
			# spin 360 and fly in a straight line along x
			if ref_x <= 2:
				ref_theta += w_ref*dt
				ref_x  += 0.1*dt
			else:
				pass

		# put theta on proper format
		while ref_theta > np.pi:
			ref_theta -= 2*np.pi
		while ref_theta < -np.pi:
			ref_theta += 2*np.pi

		ref_pose.x = ref_x
		ref_pose.y = ref_y
		ref_pose.theta = ref_theta

		print"\n Motion planner running "
		print("mode = %i" %mode)
		print("x_ref = %f" %ref_pose.x)
		print("y_ref = %f" %ref_pose.y)
		print("theta_ref = %f" %ref_pose.theta)

		pub_drone_ref_pose.publish(ref_pose)
		rate.sleep()
	
def update_mode_callback(data):
	global mode
	mode = data.data


# main
if __name__ == '__main__':
	# initialize node
	rospy.init_node('SAR_drone_motion_planner_node')
	pub_drone_ref_pose = rospy.Publisher('drone_ref_pose', Pose2D, queue_size=10)
	rospy.Subscriber('control_mode', Int32, update_mode_callback)
	rate = rospy.Rate(20) # hz

	try:
		generate_ref()
	except rospy.ROSInterruptException:
		pass