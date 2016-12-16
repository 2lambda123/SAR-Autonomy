#! /usr/bin/env python

import os
import rospy
from geometry_msgs.msg import PoseStamped

from diagnostic_msgs.msg import KeyValue
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *

#----------------------------------------------------------
# Initialize the node, register callbacks and create publishers
#----------------------------------------------------------
def start_node():
    # Initialize the node
    rospy.init_node('RandomExploration')

    # Wait for the knowledge base services to start
    print 'Waiting for knowledge base services to start'
    rospy.wait_for_service('/kcl_rosplan/get_current_instances')
    rospy.wait_for_service('/kcl_rosplan/update_knowledge_base')
    print 'Found knowledge base services. Now adding instances to knowledge base'

    # Create variables for the knoledge base services we want to use
    query_kb = rospy.ServiceProxy('/kcl_rosplan/get_current_instances', GetInstanceService)
    update_kb = rospy.ServiceProxy('/kcl_rosplan/update_knowledge_base', KnowledgeUpdateService)

    # Get all the person instances from knowledge base
    person_instances = query_kb('person').instances
    # Set goal for crate delivery
    for person_name in person_instances:
        try:
            delivered = KnowledgeItem()
            delivered.knowledge_type = KnowledgeItem.FACT
            delivered.attribute_name = 'delivered'
            delivered.values.append(KeyValue('per', person_name))
            delivered.values.append(KeyValue('co', 'supplies'))
            resp = update_kb(KnowledgeUpdateServiceRequest.ADD_GOAL, delivered)
        except rospy.ServiceException, e:
            print "Service call failed%s"%e

    # Done! No need to spin. Just wait a few moments to make sure the markers are published
    rospy.sleep(2)

#----------------------------------------------------------
# Entry point
#----------------------------------------------------------
if __name__ == '__main__':
    start_node()
