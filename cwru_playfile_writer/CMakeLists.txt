cmake_minimum_required(VERSION 2.8.3)
project(cwru_playfile_writer)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  cwru_davinci_interface
)

catkin_package(
	INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
	# DEPENDS
	# LIBRARIES
	CATKIN_DEPENDS cwru_davinci_interface roscpp
)

include_directories(${catkin_INCLUDE_DIRS})

# Executables 
add_executable(playfile_writer src/playfile_writer.cpp)
target_link_libraries(playfile_writer
  ${catkin_LIBRARIES}
)
