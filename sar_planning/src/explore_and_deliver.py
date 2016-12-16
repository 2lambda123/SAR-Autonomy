#! /usr/bin/env python

import os
import rospy
from geometry_msgs.msg import PoseStamped

from diagnostic_msgs.msg import KeyValue
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
import std_srvs.srv

#----------------------------------------------------------
# Add waypoint as goal fact
#----------------------------------------------------------
def add_goal_fact_for_turtlebot(wp_id, tbot_name):
    global query_kb, update_kb

    # Make sure that the current instances have been added to knowledge base
    try:
        wp_instances = query_kb('waypoint').instances
        tbot_instances = query_kb('turtlebot').instances
        if (wp_id not in wp_instances):
            rospy.loginfo('WARN: Waypoint instance %s not found in knowledge base'%wp_id)
        elif (tbot_name not in tbot_instances):
            rospy.loginfo('WARN: Turtlebot instance %s not found in knowledge base'%tbot_name)

        # Add goal fact
        try:
            tbot_visited = KnowledgeItem()
            tbot_visited.knowledge_type = KnowledgeItem.FACT
            tbot_visited.attribute_name = 'tbot-visited'
            tbot_visited.values.append(KeyValue('tbot', tbot_name))
            tbot_visited.values.append(KeyValue('wp', wp_id))
            resp = update_kb(KnowledgeUpdateServiceRequest.ADD_GOAL, tbot_visited)
            rospy.loginfo('Goal added for turtlebot %s'%tbot_name)

        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed while adding goal fact: %s"%e)

    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed while querying knowledge base: %s"%e)

#----------------------------------------------------------
# Initialize the node, register callbacks and create publishers
#----------------------------------------------------------
def start_node():
    global query_kb, update_kb

    # Initialize the node
    rospy.init_node('explore_and_deliver')

    # Wait for the knowledge base services to start
    rospy.loginfo('Waiting for knowledge base services to start')
    rospy.wait_for_service('/kcl_rosplan/get_current_instances')
    rospy.wait_for_service('/kcl_rosplan/update_knowledge_base')
    rospy.loginfo('Found knowledge base services. Now adding instances to knowledge base')

    # Create variables for the knoledge base services we want to use
    query_kb = rospy.ServiceProxy('/kcl_rosplan/get_current_instances', GetInstanceService)
    update_kb = rospy.ServiceProxy('/kcl_rosplan/update_knowledge_base', KnowledgeUpdateService)
    planner_service = rospy.ServiceProxy('/kcl_rosplan/planning_server', std_srvs.srv.Empty())

    # Add exploration goals
    tbot_name = 'botface' # TODO: get this from the message store
    explore_waypoints = ['wp_explore1', 'wp_explore2', 'wp_explore3']
    for wp_id in explore_waypoints:
        # Add the waypoint as a goal
        add_goal_fact_for_turtlebot(wp_id, tbot_name)

    # Set the final location (goal) of turtlebot to origin
    try:
        tbot_at = KnowledgeItem()
        tbot_at.knowledge_type = KnowledgeItem.FACT
        tbot_at.attribute_name = 'tbot-at'
        tbot_at.values.append(KeyValue('tbot', tbot_name))
        tbot_at.values.append(KeyValue('wp', 'origin'))
        resp = update_kb(KnowledgeUpdateServiceRequest.ADD_GOAL, tbot_at)
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed while setting tbot goal to origin: %s"%e)

    # Execute the planner
    planner_service()

    # Sleep to let the exploration
    print 'Exploration goals published, waiting for exploration to complete'
    rospy.loginfo('Exploration goals published, waiting for exploration to complete')
    #rospy.sleep(60)

    # Add delivery goals
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

    # Set the final location (goal) of turtlebot to origin
    try:
        tbot_at = KnowledgeItem()
        tbot_at.knowledge_type = KnowledgeItem.FACT
        tbot_at.attribute_name = 'tbot-at'
        tbot_at.values.append(KeyValue('tbot', tbot_name))
        tbot_at.values.append(KeyValue('wp', 'origin'))
        resp = update_kb(KnowledgeUpdateServiceRequest.ADD_GOAL, tbot_at)
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed while setting tbot goal to origin: %s"%e)

    # Execute the planner
    planner_service()

    print 'Delivery goals published'
    rospy.loginfo('Delivery goals published')
    # Done! No need to spin. Just wait a few moments to make sure the markers are published

#----------------------------------------------------------
# Entry point
#----------------------------------------------------------
if __name__ == '__main__':
    start_node()
