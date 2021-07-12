cmake_minimum_required(VERSION 3.5)
project(cambr_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  sensor_msgs
  std_msgs
  actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  FILES
  ToolPathConfig.msg
  HeatToolPath.msg
  CloudConfig.msg
  )

add_action_files(
   FILES
   PlanToolPaths.action
 )

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  actionlib_msgs
)

catkin_package(
    CATKIN_DEPENDS
     message_generation
     std_msgs
     sensor_msgs
     geometry_msgs
)
