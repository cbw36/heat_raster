cmake_minimum_required(VERSION 3.0.2)
project(smooth_pose_traj)

add_compile_options(-std=c++14)

find_package(Boost REQUIRED COMPONENTS system)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  std_srvs
  message_generation
  shape_msgs
  heat_msgs
)

add_definitions(${PCL_DEFINITIONS})


add_service_files(
   FILES
     SmoothPoseTrajectory.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
    shape_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    smooth_pose_traj
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    roscpp
    std_msgs
    std_srvs
    shape_msgs
    heat_msgs
  DEPENDS
    Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(smooth_pose_traj_srv src/smooth_pose_traj_srv.cpp src/smooth_pose_traj.cpp)
add_dependencies(smooth_pose_traj_srv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(smooth_pose_traj_srv ${catkin_LIBRARIES} )

add_executable(polyline_smoother_srv src/polyline_smoother_srv.cpp src/polyline_smoother.cpp)
add_dependencies(polyline_smoother_srv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(polyline_smoother_srv ${catkin_LIBRARIES} )

install(TARGETS smooth_pose_traj_srv polyline_smoother_srv
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(TARGETS polyline_smoother_srv
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

