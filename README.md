### SAR-Autonomy ###
Search and Rescue project involving a Turtlebot and an ARDrone, for the WASP Autonomous Systems Challenge 2016. 
http://wasp-sweden.org/custom/uploads/2016/11/Challenge-AutonomousSystem-2016.pdf

### External Dependencies ###
ROSPlan: https://github.com/KCL-Planning/ROSPlan 
Apriltags: https://github.com/RIVeR-Lab/apriltags_ros  
ORB-SLAM2: https://github.com/raulmur/ORB_SLAM2
ardrone_autonomy: https://github.com/AutonomyLab/ardrone_autonomy

### Installation ###
Clone the SAR-Autonomy project and external dependencies in your Catkin workspace and run catkin_make.

### Running the Demo ###
### ARDrone Demo ###
1. On a remote computer used to control the ARDrone, execute (in separate terminals)
a. roslaunch sar_drone SAR_drone_orb.launch 
b. rosrun ORB_SLAM2 Mono  [orbslamdir]/ORB_SLAM2/Vocabulary/ORBvoc.txt catkin_ws/src/sar_drone/src/ardrone_orb.yaml
c. rosrun sar_drone SAR_drone_localization.py
d. rosrun sar_drone SAR_drone_ctrl_node.py
e. rosrun sar_drone SAR_drone_motion_planner_node.py

### Turtlebot Demo ###
The ROS Master is assumed to be running on the Turtlebot computer. Since RViz consumes a lot of computing resources, the user interface must be run on a remote computer that is connected to the ROS Master running on the Turtlebot.

1. On the turtlebot computer:
a. sudo service mongodb stop (required every time you restart the computer)
b. roslaunch sar_demos sar_bringup.launch (brings up the turtlebot and gmapping)
c. roslaunch sar_demos sar_initialize.launch (brings up and initializes rosplan, action servers,etc.)

2. On the remote computer
a. roslaunch turtlebot_rviz_launchers view_navigation.launch (to visulalize stuff)
b. Add topic /kcl_rosplan/viz/…/MarkerArray to visualize goals/persons in RViz

3. On the turtlebot
a. roslaunch sar_planning mission.launch (explore the environment and deliver supplies to persons in need)

### Credits ###
WASP KTH Team 3:
Vidit Saxena
Lars Svensson
Max Åstrand
Mia Kokic
Daniel Wrang
