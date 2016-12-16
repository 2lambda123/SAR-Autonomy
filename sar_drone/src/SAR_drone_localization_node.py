#!/usr/bin/env python
#### SAR_drone_localization_node.py

import rospy
import roslib; roslib.load_manifest('sar_drone')
import tf
import numpy as np
from apriltags_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseArray 
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Pose2D 
from geometry_msgs.msg import PoseStamped 
from geometry_msgs.msg import Twist 
from wasp_custom_msgs.msg import object_loc
from optic_flow_example.msg import OpticFlowMsg



tag_id = 0
hastag_counter = 0
vx = 0
vy = 0
x_est = 0
y_est = 0

def localize():
	dt = 1.0/20.0
	global tag_id
	global tag_pose
	global drone_pose_slam
	global vx
	global vy
	global x_est 
	global y_est
	global hastag_counter
	global slam_x 
	global slam_y 
	global slam_theta 

	# set initial drone/tag pos
	tag_pose = Pose2D()
	tag_pose.x = 0.0
	tag_pose.y = 0.0	
	tag_pose.theta = 0.0

	drone_pose_slam = Pose2D()
	drone_pose_slam.x = 0
	drone_pose_slam.y = 0
	drone_pose_slam.theta = 0

	drone_pose_tag = Pose2D()
	drone_pose_tag.x = 0
	drone_pose_tag.y = 0
	drone_pose_tag.theta = 0

	drone_pose_est = Pose2D()
	drone_pose_est.x = 0
	drone_pose_est.y = 0
	drone_pose_est.theta = 0

	drone_vel_est = Twist()
	drone_vel_est.linear.x = 0
	drone_vel_est.linear.y = 0


	vxhat = 0
	vyhat = 0
	Px = 0
	Py = 0

	while not rospy.is_shutdown():


		# check input data

#		print"\n tag info: "
#		print ("tag_id = %i" % tag_id)
#		print("x_tag = %f" % tag_pose.x)
#		print("y_tag = %f" % tag_pose.y)
#		print("yaw_tag = %f" % tag_pose.theta)
		

		
		# calculate drone position relative to tag
		if tag_id == 0:
			drone_pose_tag.x = -tag_pose.x
			drone_pose_tag.y = -tag_pose.y
			drone_pose_tag.theta = -tag_pose.theta
		elif tag_id == 1:
			drone_pose_tag.x = -tag_pose.x + 2.0
			drone_pose_tag.y = -tag_pose.y + -1.0
			drone_pose_tag.theta = -tag_pose.theta
		elif tag_id == 2:
			drone_pose_tag.x = -tag_pose.x + 2.0
			drone_pose_tag.y = -tag_pose.y + 0.0
			drone_pose_tag.theta = -tag_pose.theta
		elif tag_id == 3:
			drone_pose_tag.x = -tag_pose.x - 0.0
			drone_pose_tag.y = -tag_pose.y + 1.5
			drone_pose_tag.theta = -tag_pose.theta
		elif tag_id == 4:
			drone_pose_tag.x = -tag_pose.x - 0.4
			drone_pose_tag.y = -tag_pose.y - 0.4
			drone_pose_tag.theta = -tag_pose.theta
		elif tag_id == 5:
			drone_pose_tag.x = -tag_pose.x + 0.4
			drone_pose_tag.y = -tag_pose.y + 0.4
			drone_pose_tag.theta = -tag_pose.theta
		elif tag_id == 6:
			drone_pose_tag.x = -tag_pose.x + 0.4
			drone_pose_tag.y = -tag_pose.y - 0.4
			drone_pose_tag.theta = -tag_pose.theta
		elif tag_id == 7:
			drone_pose_tag.x = -tag_pose.x +2.0- 0.4
			drone_pose_tag.y = -tag_pose.y + 0.4
			drone_pose_tag.theta = -tag_pose.theta
		elif tag_id == 8:
			drone_pose_tag.x = -tag_pose.x +2.0- 0.4
			drone_pose_tag.y = -tag_pose.y - 0.4
			drone_pose_tag.theta = -tag_pose.theta
		elif tag_id == 9:
			drone_pose_tag.x = -tag_pose.x +2.0+ 0.4
			drone_pose_tag.y = -tag_pose.y + 0.4
			drone_pose_tag.theta = -tag_pose.theta
		elif tag_id == 10:
			drone_pose_tag.x = -tag_pose.x +2.0+ 0.4
			drone_pose_tag.y = -tag_pose.y - 0.4
			drone_pose_tag.theta = -tag_pose.theta


		else:
			print("tag_id = %i " % tag_id)


#		if (hastag_counter > 5):
#			print"\n NO TAG"
#			drone_x_from_tag = (x_est + vx*d)
#			drone_y_from_tag = (y_est + vy*d)
#			drone_pose_tag.theta = 0
#		else:
#			drone_x_from_tag = drone_pose_tag.x
#			drone_y_from_tag = drone_pose_tag.y
		

		# Estimation
		last_x_est = x_est
		last_y_est = y_est
		
		if (hastag_counter <= 5): # if tag
			# calculate offset between frames
			offset_x = drone_pose_tag.x - drone_pose_slam.x
			offset_y = drone_pose_tag.y - drone_pose_slam.y
			offset_theta = drone_pose_tag.theta - drone_pose_slam.theta
			# use tag pose 
#			x_est = drone_pose_tag.x
#			y_est = drone_pose_tag.y
#			theta_est = drone_pose_tag.theta
		
		else: # if no tag
			pass

		# use offsets and slam pose
		x_est = drone_pose_slam.x + offset_x
		y_est = drone_pose_slam.y + offset_y
		theta_est = drone_pose_slam.theta + offset_theta

		drone_pose_est.x = x_est
		drone_pose_est.y = y_est
		drone_pose_est.theta = theta_est

		# velocity estimate
		Q = 1e-5
		R = 0.1
		vx = (x_est - last_x_est)/dt
		vy = (y_est - last_y_est)/dt		
		(vxhat,Px) = KF_step(vxhat, vx, Px, Q, R)
		(vyhat,Py) = KF_step(vyhat, vy, Py, Q, R)

		drone_vel_est.linear.x = vxhat
		drone_vel_est.linear.y = vyhat



#		d = 0.003
#		w1 = 0.5
#		w2 = 1.0 - w1
#		x_est = w1*(x_est + vx*d) + w2*drone_x_from_tag
#		y_est = w1*(y_est + vy*d) + w2*drone_y_from_tag
#
#		drone_vel_est.linear.x = (x_est - last_x_est)/dt
#		drone_vel_est.linear.y = (y_est -last_y_est)/dt
#
#		drone_pose_est.x = x_est
#		drone_pose_est.y = y_est
#		drone_pose_est.theta = drone_pose_tag.theta

		# publish the drone pose and velocities
		pub_drone_pose_tag.publish(drone_pose_tag)
		pub_drone_pose_est.publish(drone_pose_est)
		pub_drone_twist.publish(drone_vel_est)
		# incremnent counter
		hastag_counter += 1

		# offsets
		#x_offset =


		# prints
		print"\n drone position (tags): "
		print("x_drone = %f" % drone_pose_tag.x)
		print("y_drone = %f" % drone_pose_tag.y)
		print("yaw_drone = %f" % drone_pose_tag.theta)

		print"\n drone position (slam): "
		print("x_drone = %f" % drone_pose_slam.x)
		print("y_drone = %f" % drone_pose_slam.y)
		print("yaw_drone = %f" % drone_pose_slam.theta)

		print"\n offsets: "
		print("x:  %f" % offset_x)
		print("y:  %f" % offset_x)
		print("yaw = %f" % offset_theta)

#		print"\n optflow: "
#		print("vx = %f" % vx)
#		print("vy = %f" % vy)

		print"\n estimates: "
		print("x_est = %f" % x_est)
		print("y_est = %f" % y_est)
		print("vx_est = %f" % drone_vel_est.linear.x)
		print("vy_est = %f" % drone_vel_est.linear.y)

		rate.sleep()		


def tag_pose_callback(data):
	global tag_pose
	global hastag_counter
	tag_pose = data
	hastag_counter = 0

def tag_detection_callback(data):
	global tag_id
	# get tag id
	#print data.ID
	tag_id = data.ID

def optical_flow_callback(data):
	global vx
	global vy
	vx = np.median(data.vy, axis=0) # flipped axis in camera frame
	vy = np.median(data.vx, axis=0)

def orbslam_pose_callback(data):
	global drone_pose_slam
	drone_pose_slam.x = data.position.y * 2.5
	drone_pose_slam.y = data.position.x * 2.5

	quaternion = (
    data.orientation.x,
    data.orientation.y,
    data.orientation.z,
    data.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	drone_pose_slam.theta = euler[2]


def KF_step(xhat, z, P, Q, R):
	# time update
    xhatminus = xhat
    Pminus = P+Q

    # measurement update
    K = Pminus/( Pminus+R )
    xhat = xhatminus+K*(z-xhatminus)
    P = (1-K)*Pminus
    return (xhat,P)


# main
if __name__ == '__main__':
	# initialize node
	rospy.init_node('SAR_drone_localization_node')
	pub_drone_pose_tag = rospy.Publisher('drone_pose_tag', Pose2D, queue_size=10)
	pub_drone_pose_est = rospy.Publisher('drone_pose_est', Pose2D, queue_size=10)
	pub_drone_twist = rospy.Publisher('drone_vel_est', Twist, queue_size=10)
	#pub_loc_ok = rospy.Publisher('loc_ok', Empty, queue_size=10)
	rospy.Subscriber('tag_pose', Pose2D, tag_pose_callback)
	rospy.Subscriber('object_location', object_loc, tag_detection_callback)
	rospy.Subscriber('/optic_flow',OpticFlowMsg,optical_flow_callback)	
	rospy.Subscriber('/orbslam/pose', Pose, orbslam_pose_callback)

	#rospy.Subscriber('tag_detections', AprilTagDetectionArray, apriltag_callback)

	rate = rospy.Rate(20) # hz

	try:
		localize()
	except rospy.ROSInterruptException:
		pass