cmake_minimum_required(VERSION 2.8.3)

project(heat_ros)

add_compile_options(-std=c++14)

find_package(geometry-central REQUIRED)
find_package(Eigen3 REQUIRED)

## Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS
  actionlib
  cmake_modules
  roscpp
  heat_msgs
  smooth_pose_traj
  pcl_ros
)
find_package(PCL 1.9 REQUIRED COMPONENTS visualization)


find_package(VTK REQUIRED NO_MODULE)
if(VTK_FOUND AND ("${VTK_VERSION}" VERSION_LESS 7.1))
  message(FATAL_ERROR "The minimum required version of VTK is 7.1, but found ${VTK_VERSION}")
  set(VTK_FOUND FALSE)
else()
  include(${VTK_USE_FILE})
endif()

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
  CATKIN_DEPENDS
    actionlib
    heat_msgs
    roscpp
    smooth_pose_traj
    pcl_ros
    pcl
  DEPENDS
    VTK
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${LAPACK_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

add_executable(heat_surface_plan_server
  src/heat_surface_plan_server.cpp
  src/heat_surface_planner.cpp
  src/hm_heat_path.cpp
)

target_link_libraries(heat_surface_plan_server
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
  geometry-central::geometry-central
)
target_compile_features(heat_surface_plan_server PUBLIC cxx_std_11)

add_executable(gc_heat_method
  src/gc_heat_method.cpp)


target_link_libraries(gc_heat_method
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
  geometry-central::geometry-central
)
target_compile_features(gc_heat_method PUBLIC cxx_std_11)


add_executable(meshing_code
  src/meshing_code.cpp
  )

target_link_libraries(meshing_code
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )


#############
## Install ##
#############
#install(TARGETS  heat_surface_plan_server
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
