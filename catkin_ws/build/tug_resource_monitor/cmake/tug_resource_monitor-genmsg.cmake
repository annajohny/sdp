# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tug_resource_monitor: 2 messages, 1 services")

set(MSG_I_FLAGS "-Itug_resource_monitor:/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tug_resource_monitor_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfoArray.msg" NAME_WE)
add_custom_target(_tug_resource_monitor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tug_resource_monitor" "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfoArray.msg" "tug_resource_monitor/NodeInfo:std_msgs/Header"
)

get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfo.msg" NAME_WE)
add_custom_target(_tug_resource_monitor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tug_resource_monitor" "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfo.msg" ""
)

get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/srv/NodesInfo.srv" NAME_WE)
add_custom_target(_tug_resource_monitor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tug_resource_monitor" "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/srv/NodesInfo.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(tug_resource_monitor
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfoArray.msg"
  "${MSG_I_FLAGS}"
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfo.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tug_resource_monitor
)
_generate_msg_cpp(tug_resource_monitor
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tug_resource_monitor
)

### Generating Services
_generate_srv_cpp(tug_resource_monitor
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/srv/NodesInfo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tug_resource_monitor
)

### Generating Module File
_generate_module_cpp(tug_resource_monitor
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tug_resource_monitor
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tug_resource_monitor_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tug_resource_monitor_generate_messages tug_resource_monitor_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfoArray.msg" NAME_WE)
add_dependencies(tug_resource_monitor_generate_messages_cpp _tug_resource_monitor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfo.msg" NAME_WE)
add_dependencies(tug_resource_monitor_generate_messages_cpp _tug_resource_monitor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/srv/NodesInfo.srv" NAME_WE)
add_dependencies(tug_resource_monitor_generate_messages_cpp _tug_resource_monitor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tug_resource_monitor_gencpp)
add_dependencies(tug_resource_monitor_gencpp tug_resource_monitor_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tug_resource_monitor_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(tug_resource_monitor
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfoArray.msg"
  "${MSG_I_FLAGS}"
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfo.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tug_resource_monitor
)
_generate_msg_eus(tug_resource_monitor
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tug_resource_monitor
)

### Generating Services
_generate_srv_eus(tug_resource_monitor
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/srv/NodesInfo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tug_resource_monitor
)

### Generating Module File
_generate_module_eus(tug_resource_monitor
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tug_resource_monitor
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tug_resource_monitor_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tug_resource_monitor_generate_messages tug_resource_monitor_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfoArray.msg" NAME_WE)
add_dependencies(tug_resource_monitor_generate_messages_eus _tug_resource_monitor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfo.msg" NAME_WE)
add_dependencies(tug_resource_monitor_generate_messages_eus _tug_resource_monitor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/srv/NodesInfo.srv" NAME_WE)
add_dependencies(tug_resource_monitor_generate_messages_eus _tug_resource_monitor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tug_resource_monitor_geneus)
add_dependencies(tug_resource_monitor_geneus tug_resource_monitor_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tug_resource_monitor_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(tug_resource_monitor
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfoArray.msg"
  "${MSG_I_FLAGS}"
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfo.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tug_resource_monitor
)
_generate_msg_lisp(tug_resource_monitor
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tug_resource_monitor
)

### Generating Services
_generate_srv_lisp(tug_resource_monitor
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/srv/NodesInfo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tug_resource_monitor
)

### Generating Module File
_generate_module_lisp(tug_resource_monitor
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tug_resource_monitor
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tug_resource_monitor_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tug_resource_monitor_generate_messages tug_resource_monitor_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfoArray.msg" NAME_WE)
add_dependencies(tug_resource_monitor_generate_messages_lisp _tug_resource_monitor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfo.msg" NAME_WE)
add_dependencies(tug_resource_monitor_generate_messages_lisp _tug_resource_monitor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/srv/NodesInfo.srv" NAME_WE)
add_dependencies(tug_resource_monitor_generate_messages_lisp _tug_resource_monitor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tug_resource_monitor_genlisp)
add_dependencies(tug_resource_monitor_genlisp tug_resource_monitor_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tug_resource_monitor_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(tug_resource_monitor
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfoArray.msg"
  "${MSG_I_FLAGS}"
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfo.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tug_resource_monitor
)
_generate_msg_nodejs(tug_resource_monitor
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tug_resource_monitor
)

### Generating Services
_generate_srv_nodejs(tug_resource_monitor
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/srv/NodesInfo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tug_resource_monitor
)

### Generating Module File
_generate_module_nodejs(tug_resource_monitor
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tug_resource_monitor
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tug_resource_monitor_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tug_resource_monitor_generate_messages tug_resource_monitor_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfoArray.msg" NAME_WE)
add_dependencies(tug_resource_monitor_generate_messages_nodejs _tug_resource_monitor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfo.msg" NAME_WE)
add_dependencies(tug_resource_monitor_generate_messages_nodejs _tug_resource_monitor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/srv/NodesInfo.srv" NAME_WE)
add_dependencies(tug_resource_monitor_generate_messages_nodejs _tug_resource_monitor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tug_resource_monitor_gennodejs)
add_dependencies(tug_resource_monitor_gennodejs tug_resource_monitor_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tug_resource_monitor_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(tug_resource_monitor
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfoArray.msg"
  "${MSG_I_FLAGS}"
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfo.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tug_resource_monitor
)
_generate_msg_py(tug_resource_monitor
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tug_resource_monitor
)

### Generating Services
_generate_srv_py(tug_resource_monitor
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/srv/NodesInfo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tug_resource_monitor
)

### Generating Module File
_generate_module_py(tug_resource_monitor
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tug_resource_monitor
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tug_resource_monitor_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tug_resource_monitor_generate_messages tug_resource_monitor_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfoArray.msg" NAME_WE)
add_dependencies(tug_resource_monitor_generate_messages_py _tug_resource_monitor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg/NodeInfo.msg" NAME_WE)
add_dependencies(tug_resource_monitor_generate_messages_py _tug_resource_monitor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/srv/NodesInfo.srv" NAME_WE)
add_dependencies(tug_resource_monitor_generate_messages_py _tug_resource_monitor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tug_resource_monitor_genpy)
add_dependencies(tug_resource_monitor_genpy tug_resource_monitor_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tug_resource_monitor_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tug_resource_monitor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tug_resource_monitor
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(tug_resource_monitor_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tug_resource_monitor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tug_resource_monitor
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(tug_resource_monitor_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tug_resource_monitor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tug_resource_monitor
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(tug_resource_monitor_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tug_resource_monitor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tug_resource_monitor
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(tug_resource_monitor_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tug_resource_monitor)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tug_resource_monitor\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tug_resource_monitor
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(tug_resource_monitor_generate_messages_py std_msgs_generate_messages_py)
endif()
