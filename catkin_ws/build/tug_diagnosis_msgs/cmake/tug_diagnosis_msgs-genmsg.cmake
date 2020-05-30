# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tug_diagnosis_msgs: 6 messages, 1 services")

set(MSG_I_FLAGS "-Itug_diagnosis_msgs:/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tug_diagnosis_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/configuration.msg" NAME_WE)
add_custom_target(_tug_diagnosis_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tug_diagnosis_msgs" "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/configuration.msg" "tug_diagnosis_msgs/node_configuration:tug_diagnosis_msgs/observer_configuration"
)

get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/srv/DiagnosisConfiguration.srv" NAME_WE)
add_custom_target(_tug_diagnosis_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tug_diagnosis_msgs" "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/srv/DiagnosisConfiguration.srv" "tug_diagnosis_msgs/configuration:tug_diagnosis_msgs/node_configuration:tug_diagnosis_msgs/observer_configuration"
)

get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/resource_mode_assignement.msg" NAME_WE)
add_custom_target(_tug_diagnosis_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tug_diagnosis_msgs" "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/resource_mode_assignement.msg" ""
)

get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis_set.msg" NAME_WE)
add_custom_target(_tug_diagnosis_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tug_diagnosis_msgs" "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis_set.msg" "tug_diagnosis_msgs/resource_mode_assignement:std_msgs/Header:tug_diagnosis_msgs/diagnosis"
)

get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/node_configuration.msg" NAME_WE)
add_custom_target(_tug_diagnosis_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tug_diagnosis_msgs" "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/node_configuration.msg" ""
)

get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/observer_configuration.msg" NAME_WE)
add_custom_target(_tug_diagnosis_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tug_diagnosis_msgs" "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/observer_configuration.msg" ""
)

get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis.msg" NAME_WE)
add_custom_target(_tug_diagnosis_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tug_diagnosis_msgs" "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis.msg" "tug_diagnosis_msgs/resource_mode_assignement"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/configuration.msg"
  "${MSG_I_FLAGS}"
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/node_configuration.msg;/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/observer_configuration.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tug_diagnosis_msgs
)
_generate_msg_cpp(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/resource_mode_assignement.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tug_diagnosis_msgs
)
_generate_msg_cpp(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis_set.msg"
  "${MSG_I_FLAGS}"
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/resource_mode_assignement.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tug_diagnosis_msgs
)
_generate_msg_cpp(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/node_configuration.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tug_diagnosis_msgs
)
_generate_msg_cpp(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/observer_configuration.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tug_diagnosis_msgs
)
_generate_msg_cpp(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis.msg"
  "${MSG_I_FLAGS}"
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/resource_mode_assignement.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tug_diagnosis_msgs
)

### Generating Services
_generate_srv_cpp(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/srv/DiagnosisConfiguration.srv"
  "${MSG_I_FLAGS}"
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/configuration.msg;/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/node_configuration.msg;/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/observer_configuration.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tug_diagnosis_msgs
)

### Generating Module File
_generate_module_cpp(tug_diagnosis_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tug_diagnosis_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tug_diagnosis_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tug_diagnosis_msgs_generate_messages tug_diagnosis_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/configuration.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_cpp _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/srv/DiagnosisConfiguration.srv" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_cpp _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/resource_mode_assignement.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_cpp _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis_set.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_cpp _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/node_configuration.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_cpp _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/observer_configuration.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_cpp _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_cpp _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tug_diagnosis_msgs_gencpp)
add_dependencies(tug_diagnosis_msgs_gencpp tug_diagnosis_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tug_diagnosis_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/configuration.msg"
  "${MSG_I_FLAGS}"
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/node_configuration.msg;/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/observer_configuration.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tug_diagnosis_msgs
)
_generate_msg_eus(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/resource_mode_assignement.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tug_diagnosis_msgs
)
_generate_msg_eus(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis_set.msg"
  "${MSG_I_FLAGS}"
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/resource_mode_assignement.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tug_diagnosis_msgs
)
_generate_msg_eus(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/node_configuration.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tug_diagnosis_msgs
)
_generate_msg_eus(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/observer_configuration.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tug_diagnosis_msgs
)
_generate_msg_eus(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis.msg"
  "${MSG_I_FLAGS}"
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/resource_mode_assignement.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tug_diagnosis_msgs
)

### Generating Services
_generate_srv_eus(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/srv/DiagnosisConfiguration.srv"
  "${MSG_I_FLAGS}"
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/configuration.msg;/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/node_configuration.msg;/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/observer_configuration.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tug_diagnosis_msgs
)

### Generating Module File
_generate_module_eus(tug_diagnosis_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tug_diagnosis_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tug_diagnosis_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tug_diagnosis_msgs_generate_messages tug_diagnosis_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/configuration.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_eus _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/srv/DiagnosisConfiguration.srv" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_eus _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/resource_mode_assignement.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_eus _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis_set.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_eus _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/node_configuration.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_eus _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/observer_configuration.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_eus _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_eus _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tug_diagnosis_msgs_geneus)
add_dependencies(tug_diagnosis_msgs_geneus tug_diagnosis_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tug_diagnosis_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/configuration.msg"
  "${MSG_I_FLAGS}"
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/node_configuration.msg;/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/observer_configuration.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tug_diagnosis_msgs
)
_generate_msg_lisp(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/resource_mode_assignement.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tug_diagnosis_msgs
)
_generate_msg_lisp(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis_set.msg"
  "${MSG_I_FLAGS}"
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/resource_mode_assignement.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tug_diagnosis_msgs
)
_generate_msg_lisp(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/node_configuration.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tug_diagnosis_msgs
)
_generate_msg_lisp(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/observer_configuration.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tug_diagnosis_msgs
)
_generate_msg_lisp(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis.msg"
  "${MSG_I_FLAGS}"
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/resource_mode_assignement.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tug_diagnosis_msgs
)

### Generating Services
_generate_srv_lisp(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/srv/DiagnosisConfiguration.srv"
  "${MSG_I_FLAGS}"
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/configuration.msg;/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/node_configuration.msg;/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/observer_configuration.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tug_diagnosis_msgs
)

### Generating Module File
_generate_module_lisp(tug_diagnosis_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tug_diagnosis_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tug_diagnosis_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tug_diagnosis_msgs_generate_messages tug_diagnosis_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/configuration.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_lisp _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/srv/DiagnosisConfiguration.srv" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_lisp _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/resource_mode_assignement.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_lisp _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis_set.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_lisp _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/node_configuration.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_lisp _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/observer_configuration.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_lisp _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_lisp _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tug_diagnosis_msgs_genlisp)
add_dependencies(tug_diagnosis_msgs_genlisp tug_diagnosis_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tug_diagnosis_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/configuration.msg"
  "${MSG_I_FLAGS}"
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/node_configuration.msg;/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/observer_configuration.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tug_diagnosis_msgs
)
_generate_msg_nodejs(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/resource_mode_assignement.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tug_diagnosis_msgs
)
_generate_msg_nodejs(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis_set.msg"
  "${MSG_I_FLAGS}"
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/resource_mode_assignement.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tug_diagnosis_msgs
)
_generate_msg_nodejs(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/node_configuration.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tug_diagnosis_msgs
)
_generate_msg_nodejs(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/observer_configuration.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tug_diagnosis_msgs
)
_generate_msg_nodejs(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis.msg"
  "${MSG_I_FLAGS}"
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/resource_mode_assignement.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tug_diagnosis_msgs
)

### Generating Services
_generate_srv_nodejs(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/srv/DiagnosisConfiguration.srv"
  "${MSG_I_FLAGS}"
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/configuration.msg;/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/node_configuration.msg;/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/observer_configuration.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tug_diagnosis_msgs
)

### Generating Module File
_generate_module_nodejs(tug_diagnosis_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tug_diagnosis_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tug_diagnosis_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tug_diagnosis_msgs_generate_messages tug_diagnosis_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/configuration.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_nodejs _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/srv/DiagnosisConfiguration.srv" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_nodejs _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/resource_mode_assignement.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_nodejs _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis_set.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_nodejs _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/node_configuration.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_nodejs _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/observer_configuration.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_nodejs _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_nodejs _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tug_diagnosis_msgs_gennodejs)
add_dependencies(tug_diagnosis_msgs_gennodejs tug_diagnosis_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tug_diagnosis_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/configuration.msg"
  "${MSG_I_FLAGS}"
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/node_configuration.msg;/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/observer_configuration.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tug_diagnosis_msgs
)
_generate_msg_py(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/resource_mode_assignement.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tug_diagnosis_msgs
)
_generate_msg_py(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis_set.msg"
  "${MSG_I_FLAGS}"
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/resource_mode_assignement.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tug_diagnosis_msgs
)
_generate_msg_py(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/node_configuration.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tug_diagnosis_msgs
)
_generate_msg_py(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/observer_configuration.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tug_diagnosis_msgs
)
_generate_msg_py(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis.msg"
  "${MSG_I_FLAGS}"
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/resource_mode_assignement.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tug_diagnosis_msgs
)

### Generating Services
_generate_srv_py(tug_diagnosis_msgs
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/srv/DiagnosisConfiguration.srv"
  "${MSG_I_FLAGS}"
  "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/configuration.msg;/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/node_configuration.msg;/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/observer_configuration.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tug_diagnosis_msgs
)

### Generating Module File
_generate_module_py(tug_diagnosis_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tug_diagnosis_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tug_diagnosis_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tug_diagnosis_msgs_generate_messages tug_diagnosis_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/configuration.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_py _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/srv/DiagnosisConfiguration.srv" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_py _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/resource_mode_assignement.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_py _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis_set.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_py _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/node_configuration.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_py _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/observer_configuration.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_py _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg/diagnosis.msg" NAME_WE)
add_dependencies(tug_diagnosis_msgs_generate_messages_py _tug_diagnosis_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tug_diagnosis_msgs_genpy)
add_dependencies(tug_diagnosis_msgs_genpy tug_diagnosis_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tug_diagnosis_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tug_diagnosis_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tug_diagnosis_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(tug_diagnosis_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tug_diagnosis_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tug_diagnosis_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(tug_diagnosis_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tug_diagnosis_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tug_diagnosis_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(tug_diagnosis_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tug_diagnosis_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tug_diagnosis_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(tug_diagnosis_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tug_diagnosis_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tug_diagnosis_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tug_diagnosis_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(tug_diagnosis_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
