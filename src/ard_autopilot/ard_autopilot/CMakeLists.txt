cmake_minimum_required(VERSION 2.8.3)
project(ard_autopilot)

find_package(catkin REQUIRED COMPONENTS
  message_generation
	tf2
	tf2_ros
	roscpp
	rospy	
	cv_bridge
	geometry_msgs
  gazebo_msgs
	std_msgs
)

add_message_files(DIRECTORY msg
  FILES
  reset_req.msg
)

add_service_files(DIRECTORY srv
  FILES
  ResetPosition.srv
  SendCommand.srv
  GetRelativePose.srv
)

generate_messages(DEPENDENCIES
  std_msgs
  gazebo_msgs
)


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ard_autopilot
        CATKIN_DEPENDS tf2 tf2_ros roscpp rospy cv_bridge Toon geometry_msgs std_msgs gazebo_msgs
        )


include_directories(include)
include_directories(
	${catkin_INCLUDE_DIRS}
  
)

add_library(ard_autopilot src/offset.cpp)

add_executable(landing_autopilot src/landing_autopilot.cpp)

target_link_libraries(landing_autopilot ard_autopilot
   ${catkin_LIBRARIES}
)

list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3 -g  -ftest-coverage -fprofile-arcs")

#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB children ${CMAKE_SOURCE_DIR}/*)
FOREACH(child ${children})
  IF(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    LIST(APPEND extra_files ${dir_files})
  ENDIF()
ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})
