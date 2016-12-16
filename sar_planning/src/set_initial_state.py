#! /usr/bin/env python

import os
import rospy
from geometry_msgs.msg import PoseStamped

from diagnostic_msgs.msg import KeyValue
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
from mongodb_store.message_store import MessageStoreProxy

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

#----------------------------------------------------------
# Display marker in RViz for visualisation
#----------------------------------------------------------
def publish_goal_marker(wp_coordinates):
    global location_marker_pub, marker_id

    # Create the marker
    marker = Marker()
    marker.header.frame_id = "map"
    marker.id = marker_id;
    marker.type = 2
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 1.0 
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.pose.position.x = wp_coordinates[0]
    marker.pose.position.y = wp_coordinates[1]
    marker.lifetime = rospy.rostime.Duration()

    marker_array = MarkerArray()
    marker_array.markers.append(marker)

    location_marker_pub.publish(marker_array)

    marker_id = marker_id + 1;

#----------------------------------------------------------
# Add instance to knowledge base
#----------------------------------------------------------
def add_instance_to_knowledge_base(instance_type, instance_name):
    global query_kb, update_kb

    # Check if the current instance has already been added to knowledge base
    try:
        object_instances = query_kb(instance_type).instances
        if (instance_name not in object_instances):
            # Add the instance to the knowledge base
            try:
                new_instance = KnowledgeItem()
                new_instance.knowledge_type = KnowledgeItem.INSTANCE
                new_instance.instance_type = instance_type
                new_instance.instance_name = instance_name
                resp = update_kb(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, new_instance)
                rospy.loginfo('Instance %s added to the knowledge base'%instance_name)

            except rospy.ServiceException, e:
                rospy.loginfo("Service call failed while adding waypoint instance: %s"%e)
        else:
            rospy.loginfo('Instance %s already exists in the knowledge base, skipping'%instance_name)

    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed while querying the knowledge base: %s"%e)

#----------------------------------------------------------
# Add waypoint to scene database
#----------------------------------------------------------
def add_waypoint_to_scene_database(wp_id, wp_coordinates):
    global query_kb, update_kb

    # Construct the position object
    wp_pose = PoseStamped()
    wp_pose.header.frame_id = 'map'
    wp_pose.pose.position.x = wp_coordinates[0]
    wp_pose.pose.position.y = wp_coordinates[1]
    wp_pose.pose.position.z = wp_coordinates[2]
    wp_pose.pose.orientation.x = 0.0;
    wp_pose.pose.orientation.y = 0.0;
    wp_pose.pose.orientation.z = 1.0;
    wp_pose.pose.orientation.w = 1.0;


    # Make sure that the current instance has been added to knowledge base
    try:
        wp_instances = query_kb('waypoint').instances
        if (wp_id not in wp_instances):
            rospy.loginfo('WARN: Adding waypoint %s to scene database before instance creation in KB'%wp_id)

        msg_store = MessageStoreProxy()
        try:
            # TODO: We could check if the waypoint already exists in the database
            msg_store.insert_named(wp_id, wp_pose)
            rospy.loginfo('Location of waypoint %s added to the scene database'%wp_id)

        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed while adding waypoint to database: %s"%e)

    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed while querying knowledge base: %s"%e)



#----------------------------------------------------------
# Initialize the node, register callbacks and create publishers
#----------------------------------------------------------
def start_node():
    global query_kb, update_kb
    global location_marker_pub, marker_id 

    # Initialize the node
    rospy.init_node('sar_initialize')

    # Wait for the knowledge base services to start
    rospy.loginfo('Waiting for knowledge base services to start')
    rospy.wait_for_service('/kcl_rosplan/get_current_instances')
    rospy.wait_for_service('/kcl_rosplan/update_knowledge_base')
    rospy.loginfo('Found knowledge base services. Now adding instances and facts to knowledge base')

    # Create variables for the knoledge base services we want to use
    query_kb = rospy.ServiceProxy('/kcl_rosplan/get_current_instances', GetInstanceService)
    update_kb = rospy.ServiceProxy('/kcl_rosplan/update_knowledge_base', KnowledgeUpdateService)

    # Create a publisher to send the marker
    location_marker_pub = rospy.Publisher("/kcl_rosplan/viz/waypoints", MarkerArray, queue_size = 1)
    marker_id = 0

    # Add origin waypoint
    wp_origin = 'origin'
    add_instance_to_knowledge_base('waypoint', wp_origin)
    add_waypoint_to_scene_database(wp_origin, [0.0,0.0,0.0])

    # Add turtlebot and locate at origin
    tbot_name = 'botface'
    add_instance_to_knowledge_base('turtlebot', tbot_name)
    try:
        tbot_at = KnowledgeItem()
        tbot_at.knowledge_type = KnowledgeItem.FACT
        tbot_at.attribute_name = 'tbot-at'
        tbot_at.values.append(KeyValue('tbot', tbot_name))
        tbot_at.values.append(KeyValue('wp', wp_origin))
        resp = update_kb(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, tbot_at)
    except rospy.ServiceException, e:
        rospy.loginfo( "Service call failed%s"%e)

    # Add drone and locate at origin
    drone_name = 'mcdronald'
    add_instance_to_knowledge_base('drone', drone_name)
    try:
        drone_at = KnowledgeItem()
        drone_at.knowledge_type = KnowledgeItem.FACT
        drone_at.attribute_name = 'drone-at'
        drone_at.values.append(KeyValue('dr', drone_name))
        drone_at.values.append(KeyValue('wp', wp_origin))
        resp = update_kb(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, drone_at)
    except rospy.ServiceException, e:
        rospy.loginfo( "Service call failed%s"%e)
    # Add fact that drone is free
    try:
        drone_free = KnowledgeItem()
        drone_free.knowledge_type = KnowledgeItem.FACT
        drone_free.attribute_name = 'free'
        drone_free.values.append(KeyValue('dr', drone_name))
        resp = update_kb(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, drone_free)
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed%s"%e)


    # Initialize the crates with supplies and set the initial location origin
    content = 'supplies'
    add_instance_to_knowledge_base('content', content)
    for i in range (0, 6):
        crate_id = 'cr'+str(i)
        # Add crate instance
        add_instance_to_knowledge_base('crate', crate_id)
        # Add crate content
        try:
            contains = KnowledgeItem()
            contains.knowledge_type = KnowledgeItem.FACT
            contains.attribute_name = 'contains'
            contains.values.append(KeyValue('cr', crate_id))
            contains.values.append(KeyValue('co', content))
            resp = update_kb(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, contains)
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed%s"%e)
        # Set crate location to origin
        try:
            crate_at = KnowledgeItem()
            crate_at.knowledge_type = KnowledgeItem.FACT
            crate_at.attribute_name = 'crate-at'
            crate_at.values.append(KeyValue('cr', crate_id))
            crate_at.values.append(KeyValue('wp', wp_origin))
            resp = update_kb(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, crate_at)
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed%s"%e)

    # Create person instances
    #for i in range (0, 3):
    #    person_id = 'per'+str(i)
    #    # Add person instance
    #    add_instance_to_knowledge_base('person', person_id)
    #    # Set person location to origin
    #    try:
    #        person_at = KnowledgeItem()
    #        person_at.knowledge_type = KnowledgeItem.FACT
    #        person_at.attribute_name = 'per-at'
    #        person_at.values.append(KeyValue('per', person_id))
    #        person_at.values.append(KeyValue('wp', wp_origin))
    #        resp = update_kb(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, person_at)
    #    except rospy.ServiceException, e:
    #        print "Service call failed%s"%e

    # Done! No need to spin.

#----------------------------------------------------------
# Entry point
#----------------------------------------------------------
if __name__ == '__main__':
    start_node()
