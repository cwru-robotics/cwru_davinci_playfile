cmake_minimum_required(VERSION 2.8.3)
project(cwru_davinci_playfile)

find_package(catkin REQUIRED COMPONENTS roscpp roslib roslint cwru_davinci_kinematics uv_control)
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
	INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
	LIBRARIES playfile_format_joint playfile_format_cartesian actionlib 
	CATKIN_DEPENDS roscpp roslib roslint uv_control
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR})

roslint_cpp()


# Libraries

add_library(playfile_format_joint src/playfile_format_joint.cpp)
add_library(playfile_format_cartesian src/playfile_format_cartesian.cpp)


# executables:

add_executable(playback_jointspace src/playback_jointspace.cpp)
add_executable(playback_portalspace src/playback_portalspace.cpp)

#add_executable(record_jointspace src/record_jointspace.cpp)


target_link_libraries(playback_jointspace playfile_format_joint ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(playback_portalspace playfile_format_cartesian ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#target_link_libraries(record_jointspace
#	playfile_format_joint
#	${catkin_LIBRARIES}
#)


# tests

roslint_add_test()
