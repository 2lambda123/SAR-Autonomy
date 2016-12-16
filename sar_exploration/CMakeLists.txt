cmake_minimum_required(VERSION 2.8.3)
project(sar_exploration)

find_package(catkin REQUIRED COMPONENTS roscpp rosplan_knowledge_msgs)

catkin_package(
  CATKIN_DEPENDS rosplan_knowledge_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})
