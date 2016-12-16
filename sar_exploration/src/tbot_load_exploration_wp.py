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
# Load waypoints from file
#----------------------------------------------------------
def load_waypoints(filename):
    waypoint_array = dict()
    for line in open(filename, 'r'):
        wp_id = line.split('[')[0]
        wp_position_string = line.split('[')[1].split(']')[0]
        waypoint_array[wp_id] = wp_position_string

    return waypoint_array

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
    marker.scale.x = 0.5
    marker.scale.y = 0.5
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

    # Temporary woraround to force tbot to move to goal
    # move_base_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = 1)
    # move_base_pub.publish(wp_pose)
    # rospy.sleep(10)

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
    global location_marker_pub, marker_id 

    # Initialize the node
    rospy.init_node('RandomExploration')

    # Wait for the knowledge base services to start
    rospy.loginfo('Waiting for knowledge base and rviz services to start')
    rospy.wait_for_service('/kcl_rosplan/get_current_instances')
    rospy.wait_for_service('/kcl_rosplan/update_knowledge_base')
    #rospy.wait_for_service('/rviz/get_loggers')
    rospy.loginfo('Found knowledge base services. Now adding instances to knowledge base')

    # Create variables for the knoledge base services we want to use
    query_kb = rospy.ServiceProxy('/kcl_rosplan/get_current_instances', GetInstanceService)
    update_kb = rospy.ServiceProxy('/kcl_rosplan/update_knowledge_base', KnowledgeUpdateService)

    # Create a publisher to send the marker
    location_marker_pub = rospy.Publisher("/kcl_rosplan/viz/waypoints", MarkerArray, queue_size = 1)
    marker_id = 100

    # Load exploration waypoints from file
    path = os.path.dirname(os.path.abspath(__file__))
    explore_waypoints = load_waypoints(path + '/waypoints.txt')

    # Add exploration waypoint instances to knowledge base and their locations to scene database
    tbot_name = 'botface' # TODO: get this from the message store
    for wp_id, wp_loc_string in explore_waypoints.iteritems():
        # Extract the waypoint coordinates from the string
        wp_coordinates = map(float, wp_loc_string.split(','))
        # Create instance in knowledge base for waypoint
        add_instance_to_knowledge_base('waypoint', wp_id)
        # Add waypoint location to the database
        add_waypoint_to_scene_database(wp_id, wp_coordinates)
        # Add the waypoint as a goal
        #add_goal_fact_for_turtlebot(wp_id, tbot_name)
        # Publish marker for goal
        publish_goal_marker(wp_coordinates)

    # Set the initial location of turtlebot to origin
    try:
        tbot_at = KnowledgeItem()
        tbot_at.knowledge_type = KnowledgeItem.FACT
        tbot_at.attribute_name = 'tbot-at'
        tbot_at.values.append(KeyValue('tbot', tbot_name))
        tbot_at.values.append(KeyValue('wp', 'origin'))
        resp = update_kb(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, tbot_at)
        rospy.loginfo('turtlebot %s attached to origin'%tbot_name)
    except rospy.ServiceException, e:
        rospy.loginfo( "Service call failed while attaching tbot to origin %s"%e)

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

    # Done! No need to spin. Just wait a few moments to make sure the markers are published
    rospy.sleep(2)

#----------------------------------------------------------
# Entry point
#----------------------------------------------------------
if __name__ == '__main__':
    start_node()
