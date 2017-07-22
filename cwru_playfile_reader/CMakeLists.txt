cmake_minimum_required(VERSION 2.8.3)
project(cwru_playfile_reader)

find_package(roscpp REQUIRED)
find_package(roslib REQUIRED)
find_package(catkin REQUIRED COMPONENTS cwru_davinci_traj_streamer cwru_davinci_kinematics roslib)

#This is needed as part of the migration to ros jade and later
find_package(Eigen3)

# this is depreciated but still necessary for now.
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIR ${EIGEN_INCLUDE_DIR})
  set(EIGEN_DEP Eigen)
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
  set(EIGEN_DEP Eigen3)
endif()

#find_package(catkin_simple REQUIRED)

#catkin_simple()

include_directories(${catkin_INCLUDE_DIRS} include)
catkin_package(
	INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
	DEPENDS roscpp roslib
	LIBRARIES playfile_record
	CATKIN_DEPENDS cwru_davinci_traj_streamer cwru_davinci_kinematics roscpp roslib
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Executables
add_library(playfile_record src/playfile_record.cpp)   

add_executable(playfile_jointspace src/davinci_playfile_jointspace.cpp)
add_executable(playfile_cartspace src/davinci_playfile_cartspace_dev.cpp)
#cs_add_executable(playfile_cartspace src/davinci_playfile_cartspace_dev.cpp)
#cs_add_executable(playfile_cameraspace src/davinci_playfile_cameraspace.cpp)


target_link_libraries(playfile_jointspace playfile_record ${catkin_LIBRARIES})
target_link_libraries(playfile_cartspace playfile_record ${catkin_LIBRARIES})
#target_link_libraries(playfile_cartspace davinci_kinematics ${catkin_LIBRARIES})
#target_link_libraries(playfile_cartspace davinci_joint_publisher playfile_record davinci_kinematics ${catkin_LIBRARIES})
#target_link_libraries(playfile_cameraspace davinci_joint_publisher playfile_record davinci_kinematics ${catkin_LIBRARIES})
