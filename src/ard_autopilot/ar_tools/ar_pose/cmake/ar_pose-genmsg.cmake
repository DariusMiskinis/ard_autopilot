# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ar_pose: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iar_pose:/home/dariusm/catkin_ws/src/ar_tools/ar_pose/msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ar_pose_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/dariusm/catkin_ws/src/ar_tools/ar_pose/msg/ARMarker.msg" NAME_WE)
add_custom_target(_ar_pose_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ar_pose" "/home/dariusm/catkin_ws/src/ar_tools/ar_pose/msg/ARMarker.msg" "geometry_msgs/Point:geometry_msgs/Pose:geometry_msgs/Quaternion:std_msgs/Header:geometry_msgs/PoseWithCovariance"
)

get_filename_component(_filename "/home/dariusm/catkin_ws/src/ar_tools/ar_pose/msg/ARMarkers.msg" NAME_WE)
add_custom_target(_ar_pose_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ar_pose" "/home/dariusm/catkin_ws/src/ar_tools/ar_pose/msg/ARMarkers.msg" "geometry_msgs/Point:geometry_msgs/PoseWithCovariance:ar_pose/ARMarker:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Pose"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ar_pose
  "/home/dariusm/catkin_ws/src/ar_tools/ar_pose/msg/ARMarker.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ar_pose
)
_generate_msg_cpp(ar_pose
  "/home/dariusm/catkin_ws/src/ar_tools/ar_pose/msg/ARMarkers.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/dariusm/catkin_ws/src/ar_tools/ar_pose/msg/ARMarker.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ar_pose
)

### Generating Services

### Generating Module File
_generate_module_cpp(ar_pose
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ar_pose
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ar_pose_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ar_pose_generate_messages ar_pose_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dariusm/catkin_ws/src/ar_tools/ar_pose/msg/ARMarker.msg" NAME_WE)
add_dependencies(ar_pose_generate_messages_cpp _ar_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dariusm/catkin_ws/src/ar_tools/ar_pose/msg/ARMarkers.msg" NAME_WE)
add_dependencies(ar_pose_generate_messages_cpp _ar_pose_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ar_pose_gencpp)
add_dependencies(ar_pose_gencpp ar_pose_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ar_pose_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ar_pose
  "/home/dariusm/catkin_ws/src/ar_tools/ar_pose/msg/ARMarker.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ar_pose
)
_generate_msg_lisp(ar_pose
  "/home/dariusm/catkin_ws/src/ar_tools/ar_pose/msg/ARMarkers.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/dariusm/catkin_ws/src/ar_tools/ar_pose/msg/ARMarker.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ar_pose
)

### Generating Services

### Generating Module File
_generate_module_lisp(ar_pose
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ar_pose
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ar_pose_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ar_pose_generate_messages ar_pose_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dariusm/catkin_ws/src/ar_tools/ar_pose/msg/ARMarker.msg" NAME_WE)
add_dependencies(ar_pose_generate_messages_lisp _ar_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dariusm/catkin_ws/src/ar_tools/ar_pose/msg/ARMarkers.msg" NAME_WE)
add_dependencies(ar_pose_generate_messages_lisp _ar_pose_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ar_pose_genlisp)
add_dependencies(ar_pose_genlisp ar_pose_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ar_pose_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ar_pose
  "/home/dariusm/catkin_ws/src/ar_tools/ar_pose/msg/ARMarker.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ar_pose
)
_generate_msg_py(ar_pose
  "/home/dariusm/catkin_ws/src/ar_tools/ar_pose/msg/ARMarkers.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/dariusm/catkin_ws/src/ar_tools/ar_pose/msg/ARMarker.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ar_pose
)

### Generating Services

### Generating Module File
_generate_module_py(ar_pose
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ar_pose
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ar_pose_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ar_pose_generate_messages ar_pose_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dariusm/catkin_ws/src/ar_tools/ar_pose/msg/ARMarker.msg" NAME_WE)
add_dependencies(ar_pose_generate_messages_py _ar_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dariusm/catkin_ws/src/ar_tools/ar_pose/msg/ARMarkers.msg" NAME_WE)
add_dependencies(ar_pose_generate_messages_py _ar_pose_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ar_pose_genpy)
add_dependencies(ar_pose_genpy ar_pose_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ar_pose_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ar_pose)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ar_pose
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(ar_pose_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ar_pose)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ar_pose
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(ar_pose_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ar_pose)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ar_pose\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ar_pose
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(ar_pose_generate_messages_py geometry_msgs_generate_messages_py)
endif()
