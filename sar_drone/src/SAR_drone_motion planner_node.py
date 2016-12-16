#!/usr/bin/env python
#### SAR_drone_motion planner_node.py

import rospy
#import roslib; roslib.load_manifest('sar_drone')
import tf
import numpy as np


from geometry_msgs.msg import Pose2D 

def generate_ref():

	# set initial vals to vars

	while not rospy.is_shutdown():

		print"\n Motion planner: "
		
		rate.sleep()
		

# main
if __name__ == '__main__':
	# initialize node
	rospy.init_node('SAR_drone_motion planner_node')
	pub_drone_ref_pose = rospy.Publisher('drone_ref_pose', Pose2D, queue_size=10)

	rate = rospy.Rate(20) # hz

	try:
		generate_ref()
	except rospy.ROSInterruptException:
		pass