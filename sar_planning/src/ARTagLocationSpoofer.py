#!/usr/bin/env python
'''
This node spoofs ARTag location messages for testing
'''
import sys
import rospy
import time

import random
from random import randint

from wasp_custom_msgs.msg import object_loc

def publish_artag_location_message(tag_id):
    artag_pub = rospy.Publisher("/object_location_tb", object_loc, queue_size =1)

    artag_loc = object_loc()
    artag_loc.ID = tag_id

    # The point coordinates are between x: [2,4] and y:[-1,1]
    artag_loc.point.x = 2 + random.random()*2.0
    artag_loc.point.y = -1.0 + random.random()*2.0
    artag_loc.point.z = 0
   
    artag_pub.publish(artag_loc)

def main(args):
    rospy.init_node('ARTagLocationSpoofer')
    print 'Node initialized, publishing spoof messages to /artag_spoof_location'
    
    for i in range(0,5):
        publish_artag_location_message(i)
        rospy.sleep(5)

if __name__ == '__main__':
    main(sys.argv)
