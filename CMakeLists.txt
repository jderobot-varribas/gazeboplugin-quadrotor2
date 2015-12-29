cmake_minimum_required(VERSION 2.8.11)

project(quadrotor2)
set(libname gazebo_quadrotor2plugin)

## Dependencies
include_directories(
	${ZeroCIce_INCLUDE_DIR}
	${INTERFACES_CPP_DIR}
	${GAZEBO_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

link_directories(
	${GAZEBO_LIBRARY_DIRS}
	${INTERFACES_CPP_DIR}
)


## Project
include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Headers trick for IDEs
file(GLOB_RECURSE quadrotor2_headers "include/**")
add_library(quadrotor2_headers EXCLUDE_FROM_ALL ${quadrotor2_headers})
set_target_properties(quadrotor2_headers PROPERTIES LINKER_LANGUAGE CXX)


add_library(${libname} SHARED
	src/quadrotorplugin.cc
	src/quadrotorsensors.cc
	src/quadrotorcontrol.cc

	src/quadrotorcontrol_hector.cc
	src/pidcontroller.cc

	src/cameraproxy.cc

	src/quadrotorice.cc
	src/interfaces/pose3di.cpp
	src/interfaces/navdatai.cpp
	src/interfaces/dronecontroli.cpp
	src/interfaces/cmdveli.cpp
	src/interfaces/camerai.cpp
	src/interfaces/pushcamerai.cpp
	src/interfaces/cameraibase.cpp
)

# Gazebo
target_link_libraries(${libname} ${GAZEBO_libraries})
set_target_properties(${libname} PROPERTIES COMPILE_FLAGS "${GAZEBO_CXX_FLAGS}")

# Ice
target_link_libraries(${libname}
	${ZeroCIce_LIBRARIES}
	JderobotInterfaces
)

# Application
target_link_libraries(${libname}
	${OpenCV_LIBRARIES}
)

set_target_properties(${libname} PROPERTIES COMPILE_FLAGS "-Wall -Wextra")

## Install
INSTALL (TARGETS ${libname} DESTINATION share/jderobot/gazebo/plugins/quadrotor/)
FILE(GLOB_RECURSE CFG_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cfg)
INSTALL (FILES ${CFG_FILES} DESTINATION share/jderobot/gazebo/plugins/quadrotor/)
