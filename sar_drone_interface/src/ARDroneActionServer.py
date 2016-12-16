#!/usr/bin/env python
'''
This node implements the ARDrone actions in response to messages
from a client, e.g., the planner interface.
Currently, this node spoofs ARDrone actions by waiting for some time
and responding with a success result
'''

import rospy
import time
import actionlib

from sar_drone_interface.msg import PickCrateAction
from sar_drone_interface.msg import LoadCrateAction
from sar_drone_interface.msg import DeliverCrateAction
from sar_drone_interface.msg import FlyDroneAction
from std_msgs.msg import Empty
from std_msgs.msg import Int32

# Constant action durations (seconds)
drone_speed = 1.0; # m/s
pick_load_crate_duration = 1.0;
move_A_B_duration =  1.0;
deliver_crate_duration = 1.0;

# Implement the pick crate action to be executed by the drone
class PickCrateServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('pick_crate', PickCrateAction, self.execute, False)
        self.server.start()
        rospy.loginfo('pick_crate server started')
        self.pub_takeoff = rospy.Publisher('/drone_takeoff', Empty,queue_size=10)

    def execute(self, goal):
        # Implement the pick-crate action here
        print 'Picking crate', goal.crate_id, ' ETA: ', pick_load_crate_duration, 's'
        self.pub_takeoff.publish(Empty())
        rospy.sleep(pick_load_crate_duration)
        self.server.set_succeeded()
   
# Implement the load crate action to be executed by the drone
class LoadCrateServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('load_crate', LoadCrateAction, self.execute, False)
        self.server.start()
        rospy.loginfo('load_crate server started')
        self.pub_move_action = rospy.Publisher('/control_mode',Int32,queue_size=10)
        self.pub_takeoff = rospy.Publisher('/drone_takeoff', Empty,queue_size=10)
        self.pub_land = rospy.Publisher('/drone_land', Empty,queue_size=10)
    
    def execute(self, goal):
        # Implement the load-crate action here
        print 'Loading crate', goal.crate_id, ' ETA: ', move_A_B_duration, 's'
        # move to B
        self.pub_move_action.publish(4) # mode = 4 - go to B
        rospy.sleep(move_A_B_duration)
        
        # land at B and takeoff
        self.pub_land.publish(Empty())
        rospy.sleep(pick_load_crate_duration)
        self.pub_takeoff.publish(Empty())
        rospy.sleep(pick_load_crate_duration)

        # move to A
        self.pub_move_action.publish(5) # mode = 5 - go to A
        rospy.sleep(move_A_B_duration)
        
        # land at A
        self.pub_land.publish(Empty())
        self.server.set_succeeded()
   
# Implement the pick crate action to be executed by the drone
class DeliverCrateServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('deliver_crate', DeliverCrateAction, self.execute, False)
        self.server.start()
        rospy.loginfo('deliver_crate server started')

    def execute(self, goal):
        # Implement the deliver-crate action here
        print 'Delivering crate', goal.crate_id, ' ETA: ', deliver_crate_duration, 's'
        rospy.sleep(deliver_crate_duration)
        self.server.set_succeeded()
   
# Implement the fly drone action to be executed by the drone
class FlyDroneServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('fly_drone', FlyDroneAction, self.execute, False)
        self.server.start()
        print 'fly_drone server started'
        

    def execute(self, goal):
        # Implement the fly-drone action here
        start_pos = goal.start_pose.position
        end_pos = goal.target_pose.position
        fly_distance = (start_pos.x - end_pos.x)**2 + (start_pos.y - end_pos.y)**2
        fly_duration = fly_distance/drone_speed
        print 'Flying drone to [', end_pos.x, ',' ,end_pos.y, '] ETA: ', fly_duration, 's'
        rospy.sleep(fly_duration)
   
if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('ARDroneActionServer')

    # Instantiate and start the servers
    pick_crate_server = PickCrateServer()
    load_crate_server = LoadCrateServer()
    deliver_crate_server = DeliverCrateServer()
#    fly_drone_server = FlyDroneServer()

    rospy.spin() 
