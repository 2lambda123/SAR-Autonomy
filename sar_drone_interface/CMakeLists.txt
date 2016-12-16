cmake_minimum_required(VERSION 2.8.3)
project(sar_drone_interface)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  actionlib_msgs
  actionlib
  geometry_msgs
)

add_action_files(
  DIRECTORY
  action
  FILES
  PickCrate.action
  LoadCrate.action
  DeliverCrate.action
  FlyDrone.action
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  actionlib_msgs
)
