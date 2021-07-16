cmake_minimum_required(VERSION 2.8.3)

project(heat_ros)
set(CMAKE_FIND_DEBUG_MODE 1)

add_compile_options(-std=c++14)

find_package(geometry-central REQUIRED)
find_package(Eigen3 REQUIRED)

## Find catkin macros and libraries
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake") # look for stuff in the /cmake directory
find_package(catkin REQUIRED COMPONENTS
  actionlib
  cmake_modules
  roscpp
  heat_msgs
  smooth_pose_traj
  pcl_ros
  visualization_msgs
  geometric_shapes
  swri_profiler
  opp_gui
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
    visualization_msgs
    geometric_shapes
    swri_profiler
    opp_gui
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


add_executable(heat_surface_plan_client
  src/heat_surface_plan_client.cpp
)
target_link_libraries(heat_surface_plan_client
  ${catkin_LIBRARIES}
  Qt5::Core
)
target_compile_features(heat_surface_plan_client PUBLIC cxx_std_11)


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

add_executable(pcl_pc_to_mesh
  src/pcl_pc_to_mesh.cpp)
target_link_libraries(pcl_pc_to_mesh
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
  geometry-central::geometry-central
)
target_compile_features(pcl_pc_to_mesh PUBLIC cxx_std_11)


add_executable(gc_pc_to_mesh
  src/gc_pc_to_mesh.cpp)
target_link_libraries(gc_pc_to_mesh
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
  geometry-central::geometry-central
)
target_compile_features(gc_pc_to_mesh PUBLIC cxx_std_11)


add_executable(pcl_pc_to_mesh_no_removal
  src/pcl_pc_to_mesh_no_removal.cpp)
target_link_libraries(pcl_pc_to_mesh_no_removal
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
  geometry-central::geometry-central
)
target_compile_features(pcl_pc_to_mesh_no_removal PUBLIC cxx_std_11)

add_executable(create_planar_pc
  src/create_planar_pc.cpp)
target_link_libraries(create_planar_pc
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
  geometry-central::geometry-central
)
target_compile_features(create_planar_pc PUBLIC cxx_std_11)


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
