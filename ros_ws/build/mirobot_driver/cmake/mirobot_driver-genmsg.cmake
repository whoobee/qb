# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "mirobot_driver: 3 messages, 0 services")

set(MSG_I_FLAGS "-Imirobot_driver:/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(mirobot_driver_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/wheel_telemetry.msg" NAME_WE)
add_custom_target(_mirobot_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mirobot_driver" "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/wheel_telemetry.msg" ""
)

get_filename_component(_filename "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/bot_telemetry.msg" NAME_WE)
add_custom_target(_mirobot_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mirobot_driver" "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/bot_telemetry.msg" ""
)

get_filename_component(_filename "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/wheel_control.msg" NAME_WE)
add_custom_target(_mirobot_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mirobot_driver" "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/wheel_control.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(mirobot_driver
  "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/wheel_telemetry.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirobot_driver
)
_generate_msg_cpp(mirobot_driver
  "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/bot_telemetry.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirobot_driver
)
_generate_msg_cpp(mirobot_driver
  "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/wheel_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirobot_driver
)

### Generating Services

### Generating Module File
_generate_module_cpp(mirobot_driver
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirobot_driver
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(mirobot_driver_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(mirobot_driver_generate_messages mirobot_driver_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/wheel_telemetry.msg" NAME_WE)
add_dependencies(mirobot_driver_generate_messages_cpp _mirobot_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/bot_telemetry.msg" NAME_WE)
add_dependencies(mirobot_driver_generate_messages_cpp _mirobot_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/wheel_control.msg" NAME_WE)
add_dependencies(mirobot_driver_generate_messages_cpp _mirobot_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mirobot_driver_gencpp)
add_dependencies(mirobot_driver_gencpp mirobot_driver_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mirobot_driver_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(mirobot_driver
  "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/wheel_telemetry.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirobot_driver
)
_generate_msg_eus(mirobot_driver
  "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/bot_telemetry.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirobot_driver
)
_generate_msg_eus(mirobot_driver
  "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/wheel_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirobot_driver
)

### Generating Services

### Generating Module File
_generate_module_eus(mirobot_driver
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirobot_driver
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(mirobot_driver_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(mirobot_driver_generate_messages mirobot_driver_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/wheel_telemetry.msg" NAME_WE)
add_dependencies(mirobot_driver_generate_messages_eus _mirobot_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/bot_telemetry.msg" NAME_WE)
add_dependencies(mirobot_driver_generate_messages_eus _mirobot_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/wheel_control.msg" NAME_WE)
add_dependencies(mirobot_driver_generate_messages_eus _mirobot_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mirobot_driver_geneus)
add_dependencies(mirobot_driver_geneus mirobot_driver_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mirobot_driver_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(mirobot_driver
  "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/wheel_telemetry.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirobot_driver
)
_generate_msg_lisp(mirobot_driver
  "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/bot_telemetry.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirobot_driver
)
_generate_msg_lisp(mirobot_driver
  "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/wheel_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirobot_driver
)

### Generating Services

### Generating Module File
_generate_module_lisp(mirobot_driver
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirobot_driver
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(mirobot_driver_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(mirobot_driver_generate_messages mirobot_driver_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/wheel_telemetry.msg" NAME_WE)
add_dependencies(mirobot_driver_generate_messages_lisp _mirobot_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/bot_telemetry.msg" NAME_WE)
add_dependencies(mirobot_driver_generate_messages_lisp _mirobot_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/wheel_control.msg" NAME_WE)
add_dependencies(mirobot_driver_generate_messages_lisp _mirobot_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mirobot_driver_genlisp)
add_dependencies(mirobot_driver_genlisp mirobot_driver_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mirobot_driver_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(mirobot_driver
  "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/wheel_telemetry.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirobot_driver
)
_generate_msg_nodejs(mirobot_driver
  "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/bot_telemetry.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirobot_driver
)
_generate_msg_nodejs(mirobot_driver
  "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/wheel_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirobot_driver
)

### Generating Services

### Generating Module File
_generate_module_nodejs(mirobot_driver
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirobot_driver
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(mirobot_driver_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(mirobot_driver_generate_messages mirobot_driver_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/wheel_telemetry.msg" NAME_WE)
add_dependencies(mirobot_driver_generate_messages_nodejs _mirobot_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/bot_telemetry.msg" NAME_WE)
add_dependencies(mirobot_driver_generate_messages_nodejs _mirobot_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/wheel_control.msg" NAME_WE)
add_dependencies(mirobot_driver_generate_messages_nodejs _mirobot_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mirobot_driver_gennodejs)
add_dependencies(mirobot_driver_gennodejs mirobot_driver_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mirobot_driver_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(mirobot_driver
  "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/wheel_telemetry.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirobot_driver
)
_generate_msg_py(mirobot_driver
  "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/bot_telemetry.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirobot_driver
)
_generate_msg_py(mirobot_driver
  "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/wheel_control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirobot_driver
)

### Generating Services

### Generating Module File
_generate_module_py(mirobot_driver
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirobot_driver
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(mirobot_driver_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(mirobot_driver_generate_messages mirobot_driver_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/wheel_telemetry.msg" NAME_WE)
add_dependencies(mirobot_driver_generate_messages_py _mirobot_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/bot_telemetry.msg" NAME_WE)
add_dependencies(mirobot_driver_generate_messages_py _mirobot_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg/wheel_control.msg" NAME_WE)
add_dependencies(mirobot_driver_generate_messages_py _mirobot_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mirobot_driver_genpy)
add_dependencies(mirobot_driver_genpy mirobot_driver_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mirobot_driver_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirobot_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mirobot_driver
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(mirobot_driver_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirobot_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mirobot_driver
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(mirobot_driver_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirobot_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mirobot_driver
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(mirobot_driver_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirobot_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mirobot_driver
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(mirobot_driver_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirobot_driver)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirobot_driver\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mirobot_driver
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(mirobot_driver_generate_messages_py std_msgs_generate_messages_py)
endif()
