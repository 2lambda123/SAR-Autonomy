#!/usr/bin/env python
'''
This node handles ARTag and Person detection messages
'''
import os
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

from diagnostic_msgs.msg import KeyValue
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
from mongodb_store.message_store import MessageStoreProxy

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

from nav_msgs.msg import Odometry

from wasp_custom_msgs.msg import object_loc

#----------------------------------------------------------
# Dictionary to store ARTag-person mappings as key-value pairs
#----------------------------------------------------------
artag_person_dict = dict()
marker_colors = {'ARTag': [0.0, 0.0 ,1.0],
                 'Person': [0.0, 1.0, 0.0]}
marker_id = 1000 # Increment ID for each marker
tbot_pose = Point()

def tbot_odom_callback(data):
    global tbot_pose
    tbot_pose = data.pose.pose.position

#----------------------------------------------------------
# Load ARTag ID to person ID mapping from file
#----------------------------------------------------------
def load_artag_person_mapping(filename):
    global artag_person_dict
    with open(filename, 'r') as f:
        for line in f:
            artag_id = line.split(',')[0]
            person_id = line.split(',')[1]
            artag_person_dict[artag_id] = person_id

#----------------------------------------------------------
# Display marker in RViz for visualisation
#----------------------------------------------------------
def publish_rviz_marker(marker_type, location):
    global marker_colors, marker_id
    global location_marker_pub

    color = marker_colors[marker_type]

    marker_array = MarkerArray()

    marker = Marker()
    marker.header.frame_id = "map"
    marker.id = marker_id;
    marker.type = 2
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.pose.position.x = location.x
    marker.pose.position.y = location.y
    marker.lifetime = rospy.rostime.Duration()

    marker_array.markers.append(marker)
    location_marker_pub.publish(marker_array) 

    marker_id = marker_id + 1;
    
#----------------------------------------------------------
# Update ROSPlan knowledge base when AR Tag is detected
#----------------------------------------------------------
def artag_location_callback(data):

    global artag_person_dict

    # Extract the ARTag info and construct its name for the knowledge base
    artag_id = data.ID
    artag_loc = data.point
    artag_name = 'wp_artag' + str(artag_id)

    print 'Received location for ARTag ID: ', data.ID

    add_artag_to_knowledge_base(artag_name, artag_loc)

    #if str(artag_id) in artag_person_dict:
    #    person_id = artag_person_dict[str(artag_id)]
    #    person_name = 'per' + person_id
    #    add_person_to_knowledge_base(person_name, artag_name)

    person_name = 'per' + str(artag_id)
    add_person_to_knowledge_base(person_name, artag_name)

#----------------------------------------------------------
# Add ARTag Location to knowledge base and scene database
#----------------------------------------------------------
def add_artag_to_knowledge_base(artag_name, artag_loc):
    global query_kb, update_kb
    global location_markers_pub

    # Check if the current AR Tag has already been added to knowledge base
    try:
        waypoint_instances = query_kb('waypoint').instances 
        if (artag_name not in waypoint_instances):
            # Add the ARTag to the knowledge base
            try:
                wp_artag = KnowledgeItem()
                wp_artag.knowledge_type = KnowledgeItem.INSTANCE
                wp_artag.instance_type = 'waypoint'
                wp_artag.instance_name = artag_name
                resp = update_kb(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, wp_artag)
                print 'ARTag: ', artag_name, ' added to the knowledge base'
                #publish_rviz_marker('ARTag', artag_loc)

            except rospy.ServiceException, e:
                print "Service call failed while adding waypoint instance: %s"%e

            # Add ARTag location to the scene database
            msg_store = MessageStoreProxy()
            try:
                artag_pose = PoseStamped()
                artag_pose.header.frame_id = 'map'
                artag_pose.header.stamp = rospy.get_rostime()
                artag_pose.pose.position.x = artag_loc.x 
                artag_pose.pose.position.y = artag_loc.y
                artag_pose.pose.orientation.z = 1.0 
                artag_pose.pose.orientation.w = 1.0 
                msg_store.insert_named(artag_name, artag_pose)
                print 'Location of ARTag ID: ', artag_name, ' added to the scene database'
            except rospy.ServiceException, e:
                print "Service call failed while adding waypoint to database: %s"%e
        else:
             print 'ARTag already in knowledge base, skipping'
             #TODO: you might still want to update the location 
             #      of ARTag in the scene database

    except rospy.ServiceException, e:
        print "Service call failed while querying the knowledge base: %s"%e

#----------------------------------------------------------
# Add Person to knowledge base and attach to location
#----------------------------------------------------------
def add_person_to_knowledge_base(person_name, artag_name):
    global query_kb, update_kb
    global location_markers_pub

    # Check if the person already exists in the knowledge base
    try:
        person_instances = query_kb('person').instances
        if (person_name not in person_instances):
            # Create person instance
            try:
                person = KnowledgeItem()
                person.knowledge_type = KnowledgeItem.INSTANCE
                person.instance_type = 'person'
                person.instance_name = person_name
                resp = update_kb(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, person)
                print 'person ', person_name, ' added to the knowledge base'
            except rospy.ServiceException, e:
                print "Service call failed while adding waypoint instance: %s"%e

            # Attach person to location
            try:
                person_at = KnowledgeItem()
                person_at.knowledge_type = KnowledgeItem.FACT
                person_at.attribute_name = 'per-at'
                person_at.values.append(KeyValue('per', person_name))
                person_at.values.append(KeyValue('wp', artag_name))
                resp = update_kb(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, person_at)
                print 'person ', person_name, ' attached to location ', artag_name
                
                # Get location from database
                msg_store = MessageStoreProxy()
                artag_pose = msg_store.query_named(artag_name, PoseStamped._type)
                # Publish person location to RViz
                publish_rviz_marker('Person', artag_pose[0].pose.position)

            except rospy.ServiceException, e:
                print "Service call failed while attaching person to location: %s"%e

        else:
            print 'Person already in knowledge base, skipping'

    except rospy.ServiceException, e:
        print "Service call failed while querying the knowledge base: %s"%e

#----------------------------------------------------------
# Initialize the node, register callbacks and create publishers
#----------------------------------------------------------
def start_node():
    global query_kb, update_kb
    global location_marker_pub
    
    # Initialize the node
    rospy.init_node('LocationHandler')

    # Sleep 5s to let rviz start up
    rospy.sleep(5)

    # Wait for the knowledge base services to start
    print 'Waiting for knowledge base services to start'
    rospy.wait_for_service('/kcl_rosplan/get_current_instances')
    rospy.wait_for_service('/kcl_rosplan/update_knowledge_base')

    # Create variables for the knoledge base services we want to use
    query_kb = rospy.ServiceProxy('/kcl_rosplan/get_current_instances', GetInstanceService)
    update_kb = rospy.ServiceProxy('/kcl_rosplan/update_knowledge_base', KnowledgeUpdateService)

    # Register callback for ARTag location messages
    print 'Found knowledge base services. Registering callbacks'
    #rospy.Subscriber("/artag_spoof_location", object_loc, artag_location_callback)
    rospy.Subscriber("/object_location_tb", object_loc, artag_location_callback)
    rospy.Subscriber("/odom", Odometry, tbot_odom_callback)

    # Load the ARTag to person mapping from file
    path = os.path.dirname(os.path.abspath(__file__))
    load_artag_person_mapping(path + '/ARTagPersonID.txt')

    # Create a publisher to visualize ARTag and person locations in RViz
    location_marker_pub = rospy.Publisher("/kcl_rosplan/viz/waypoints", MarkerArray, queue_size = 1)
  
    rospy.spin()

#----------------------------------------------------------
# Entry point
#----------------------------------------------------------
if __name__ == '__main__':
    start_node()
