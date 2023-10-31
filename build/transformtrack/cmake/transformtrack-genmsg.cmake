# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "transformtrack: 0 messages, 2 services")

set(MSG_I_FLAGS "-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(transformtrack_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sebastien/ProjetVA50/VA50-navigation-main/src/transformtrack/srv/DropVelocity.srv" NAME_WE)
add_custom_target(_transformtrack_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "transformtrack" "/home/sebastien/ProjetVA50/VA50-navigation-main/src/transformtrack/srv/DropVelocity.srv" ""
)

get_filename_component(_filename "/home/sebastien/ProjetVA50/VA50-navigation-main/src/transformtrack/srv/TransformBatch.srv" NAME_WE)
add_custom_target(_transformtrack_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "transformtrack" "/home/sebastien/ProjetVA50/VA50-navigation-main/src/transformtrack/srv/TransformBatch.srv" "std_msgs/MultiArrayLayout:std_msgs/MultiArrayDimension:std_msgs/Float64MultiArray"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(transformtrack
  "/home/sebastien/ProjetVA50/VA50-navigation-main/src/transformtrack/srv/DropVelocity.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transformtrack
)
_generate_srv_cpp(transformtrack
  "/home/sebastien/ProjetVA50/VA50-navigation-main/src/transformtrack/srv/TransformBatch.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transformtrack
)

### Generating Module File
_generate_module_cpp(transformtrack
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transformtrack
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(transformtrack_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(transformtrack_generate_messages transformtrack_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sebastien/ProjetVA50/VA50-navigation-main/src/transformtrack/srv/DropVelocity.srv" NAME_WE)
add_dependencies(transformtrack_generate_messages_cpp _transformtrack_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastien/ProjetVA50/VA50-navigation-main/src/transformtrack/srv/TransformBatch.srv" NAME_WE)
add_dependencies(transformtrack_generate_messages_cpp _transformtrack_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(transformtrack_gencpp)
add_dependencies(transformtrack_gencpp transformtrack_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS transformtrack_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(transformtrack
  "/home/sebastien/ProjetVA50/VA50-navigation-main/src/transformtrack/srv/DropVelocity.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transformtrack
)
_generate_srv_eus(transformtrack
  "/home/sebastien/ProjetVA50/VA50-navigation-main/src/transformtrack/srv/TransformBatch.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transformtrack
)

### Generating Module File
_generate_module_eus(transformtrack
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transformtrack
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(transformtrack_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(transformtrack_generate_messages transformtrack_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sebastien/ProjetVA50/VA50-navigation-main/src/transformtrack/srv/DropVelocity.srv" NAME_WE)
add_dependencies(transformtrack_generate_messages_eus _transformtrack_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastien/ProjetVA50/VA50-navigation-main/src/transformtrack/srv/TransformBatch.srv" NAME_WE)
add_dependencies(transformtrack_generate_messages_eus _transformtrack_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(transformtrack_geneus)
add_dependencies(transformtrack_geneus transformtrack_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS transformtrack_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(transformtrack
  "/home/sebastien/ProjetVA50/VA50-navigation-main/src/transformtrack/srv/DropVelocity.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transformtrack
)
_generate_srv_lisp(transformtrack
  "/home/sebastien/ProjetVA50/VA50-navigation-main/src/transformtrack/srv/TransformBatch.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transformtrack
)

### Generating Module File
_generate_module_lisp(transformtrack
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transformtrack
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(transformtrack_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(transformtrack_generate_messages transformtrack_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sebastien/ProjetVA50/VA50-navigation-main/src/transformtrack/srv/DropVelocity.srv" NAME_WE)
add_dependencies(transformtrack_generate_messages_lisp _transformtrack_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastien/ProjetVA50/VA50-navigation-main/src/transformtrack/srv/TransformBatch.srv" NAME_WE)
add_dependencies(transformtrack_generate_messages_lisp _transformtrack_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(transformtrack_genlisp)
add_dependencies(transformtrack_genlisp transformtrack_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS transformtrack_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(transformtrack
  "/home/sebastien/ProjetVA50/VA50-navigation-main/src/transformtrack/srv/DropVelocity.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transformtrack
)
_generate_srv_nodejs(transformtrack
  "/home/sebastien/ProjetVA50/VA50-navigation-main/src/transformtrack/srv/TransformBatch.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transformtrack
)

### Generating Module File
_generate_module_nodejs(transformtrack
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transformtrack
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(transformtrack_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(transformtrack_generate_messages transformtrack_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sebastien/ProjetVA50/VA50-navigation-main/src/transformtrack/srv/DropVelocity.srv" NAME_WE)
add_dependencies(transformtrack_generate_messages_nodejs _transformtrack_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastien/ProjetVA50/VA50-navigation-main/src/transformtrack/srv/TransformBatch.srv" NAME_WE)
add_dependencies(transformtrack_generate_messages_nodejs _transformtrack_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(transformtrack_gennodejs)
add_dependencies(transformtrack_gennodejs transformtrack_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS transformtrack_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(transformtrack
  "/home/sebastien/ProjetVA50/VA50-navigation-main/src/transformtrack/srv/DropVelocity.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transformtrack
)
_generate_srv_py(transformtrack
  "/home/sebastien/ProjetVA50/VA50-navigation-main/src/transformtrack/srv/TransformBatch.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transformtrack
)

### Generating Module File
_generate_module_py(transformtrack
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transformtrack
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(transformtrack_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(transformtrack_generate_messages transformtrack_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sebastien/ProjetVA50/VA50-navigation-main/src/transformtrack/srv/DropVelocity.srv" NAME_WE)
add_dependencies(transformtrack_generate_messages_py _transformtrack_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastien/ProjetVA50/VA50-navigation-main/src/transformtrack/srv/TransformBatch.srv" NAME_WE)
add_dependencies(transformtrack_generate_messages_py _transformtrack_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(transformtrack_genpy)
add_dependencies(transformtrack_genpy transformtrack_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS transformtrack_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transformtrack)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transformtrack
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(transformtrack_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transformtrack)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transformtrack
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(transformtrack_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transformtrack)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transformtrack
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(transformtrack_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transformtrack)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transformtrack
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(transformtrack_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transformtrack)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transformtrack\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transformtrack
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(transformtrack_generate_messages_py geometry_msgs_generate_messages_py)
endif()
