# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "fov: 2 messages, 3 services")

set(MSG_I_FLAGS "-Ifov:/home/icvl/FOMA/src/fov/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(fov_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/icvl/FOMA/src/fov/msg/FishState.msg" NAME_WE)
add_custom_target(_fov_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fov" "/home/icvl/FOMA/src/fov/msg/FishState.msg" ""
)

get_filename_component(_filename "/home/icvl/FOMA/src/fov/msg/FomaLocation.msg" NAME_WE)
add_custom_target(_fov_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fov" "/home/icvl/FOMA/src/fov/msg/FomaLocation.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/icvl/FOMA/src/fov/srv/Check.srv" NAME_WE)
add_custom_target(_fov_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fov" "/home/icvl/FOMA/src/fov/srv/Check.srv" ""
)

get_filename_component(_filename "/home/icvl/FOMA/src/fov/srv/Light.srv" NAME_WE)
add_custom_target(_fov_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fov" "/home/icvl/FOMA/src/fov/srv/Light.srv" ""
)

get_filename_component(_filename "/home/icvl/FOMA/src/fov/srv/Coordinate.srv" NAME_WE)
add_custom_target(_fov_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fov" "/home/icvl/FOMA/src/fov/srv/Coordinate.srv" "std_msgs/MultiArrayLayout:std_msgs/MultiArrayDimension:std_msgs/UInt8MultiArray"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(fov
  "/home/icvl/FOMA/src/fov/msg/FishState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fov
)
_generate_msg_cpp(fov
  "/home/icvl/FOMA/src/fov/msg/FomaLocation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fov
)

### Generating Services
_generate_srv_cpp(fov
  "/home/icvl/FOMA/src/fov/srv/Check.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fov
)
_generate_srv_cpp(fov
  "/home/icvl/FOMA/src/fov/srv/Light.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fov
)
_generate_srv_cpp(fov
  "/home/icvl/FOMA/src/fov/srv/Coordinate.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt8MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fov
)

### Generating Module File
_generate_module_cpp(fov
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fov
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(fov_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(fov_generate_messages fov_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/icvl/FOMA/src/fov/msg/FishState.msg" NAME_WE)
add_dependencies(fov_generate_messages_cpp _fov_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/icvl/FOMA/src/fov/msg/FomaLocation.msg" NAME_WE)
add_dependencies(fov_generate_messages_cpp _fov_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/icvl/FOMA/src/fov/srv/Check.srv" NAME_WE)
add_dependencies(fov_generate_messages_cpp _fov_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/icvl/FOMA/src/fov/srv/Light.srv" NAME_WE)
add_dependencies(fov_generate_messages_cpp _fov_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/icvl/FOMA/src/fov/srv/Coordinate.srv" NAME_WE)
add_dependencies(fov_generate_messages_cpp _fov_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fov_gencpp)
add_dependencies(fov_gencpp fov_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fov_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(fov
  "/home/icvl/FOMA/src/fov/msg/FishState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fov
)
_generate_msg_eus(fov
  "/home/icvl/FOMA/src/fov/msg/FomaLocation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fov
)

### Generating Services
_generate_srv_eus(fov
  "/home/icvl/FOMA/src/fov/srv/Check.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fov
)
_generate_srv_eus(fov
  "/home/icvl/FOMA/src/fov/srv/Light.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fov
)
_generate_srv_eus(fov
  "/home/icvl/FOMA/src/fov/srv/Coordinate.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt8MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fov
)

### Generating Module File
_generate_module_eus(fov
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fov
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(fov_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(fov_generate_messages fov_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/icvl/FOMA/src/fov/msg/FishState.msg" NAME_WE)
add_dependencies(fov_generate_messages_eus _fov_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/icvl/FOMA/src/fov/msg/FomaLocation.msg" NAME_WE)
add_dependencies(fov_generate_messages_eus _fov_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/icvl/FOMA/src/fov/srv/Check.srv" NAME_WE)
add_dependencies(fov_generate_messages_eus _fov_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/icvl/FOMA/src/fov/srv/Light.srv" NAME_WE)
add_dependencies(fov_generate_messages_eus _fov_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/icvl/FOMA/src/fov/srv/Coordinate.srv" NAME_WE)
add_dependencies(fov_generate_messages_eus _fov_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fov_geneus)
add_dependencies(fov_geneus fov_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fov_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(fov
  "/home/icvl/FOMA/src/fov/msg/FishState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fov
)
_generate_msg_lisp(fov
  "/home/icvl/FOMA/src/fov/msg/FomaLocation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fov
)

### Generating Services
_generate_srv_lisp(fov
  "/home/icvl/FOMA/src/fov/srv/Check.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fov
)
_generate_srv_lisp(fov
  "/home/icvl/FOMA/src/fov/srv/Light.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fov
)
_generate_srv_lisp(fov
  "/home/icvl/FOMA/src/fov/srv/Coordinate.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt8MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fov
)

### Generating Module File
_generate_module_lisp(fov
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fov
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(fov_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(fov_generate_messages fov_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/icvl/FOMA/src/fov/msg/FishState.msg" NAME_WE)
add_dependencies(fov_generate_messages_lisp _fov_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/icvl/FOMA/src/fov/msg/FomaLocation.msg" NAME_WE)
add_dependencies(fov_generate_messages_lisp _fov_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/icvl/FOMA/src/fov/srv/Check.srv" NAME_WE)
add_dependencies(fov_generate_messages_lisp _fov_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/icvl/FOMA/src/fov/srv/Light.srv" NAME_WE)
add_dependencies(fov_generate_messages_lisp _fov_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/icvl/FOMA/src/fov/srv/Coordinate.srv" NAME_WE)
add_dependencies(fov_generate_messages_lisp _fov_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fov_genlisp)
add_dependencies(fov_genlisp fov_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fov_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(fov
  "/home/icvl/FOMA/src/fov/msg/FishState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fov
)
_generate_msg_nodejs(fov
  "/home/icvl/FOMA/src/fov/msg/FomaLocation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fov
)

### Generating Services
_generate_srv_nodejs(fov
  "/home/icvl/FOMA/src/fov/srv/Check.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fov
)
_generate_srv_nodejs(fov
  "/home/icvl/FOMA/src/fov/srv/Light.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fov
)
_generate_srv_nodejs(fov
  "/home/icvl/FOMA/src/fov/srv/Coordinate.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt8MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fov
)

### Generating Module File
_generate_module_nodejs(fov
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fov
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(fov_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(fov_generate_messages fov_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/icvl/FOMA/src/fov/msg/FishState.msg" NAME_WE)
add_dependencies(fov_generate_messages_nodejs _fov_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/icvl/FOMA/src/fov/msg/FomaLocation.msg" NAME_WE)
add_dependencies(fov_generate_messages_nodejs _fov_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/icvl/FOMA/src/fov/srv/Check.srv" NAME_WE)
add_dependencies(fov_generate_messages_nodejs _fov_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/icvl/FOMA/src/fov/srv/Light.srv" NAME_WE)
add_dependencies(fov_generate_messages_nodejs _fov_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/icvl/FOMA/src/fov/srv/Coordinate.srv" NAME_WE)
add_dependencies(fov_generate_messages_nodejs _fov_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fov_gennodejs)
add_dependencies(fov_gennodejs fov_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fov_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(fov
  "/home/icvl/FOMA/src/fov/msg/FishState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fov
)
_generate_msg_py(fov
  "/home/icvl/FOMA/src/fov/msg/FomaLocation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fov
)

### Generating Services
_generate_srv_py(fov
  "/home/icvl/FOMA/src/fov/srv/Check.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fov
)
_generate_srv_py(fov
  "/home/icvl/FOMA/src/fov/srv/Light.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fov
)
_generate_srv_py(fov
  "/home/icvl/FOMA/src/fov/srv/Coordinate.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt8MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fov
)

### Generating Module File
_generate_module_py(fov
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fov
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(fov_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(fov_generate_messages fov_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/icvl/FOMA/src/fov/msg/FishState.msg" NAME_WE)
add_dependencies(fov_generate_messages_py _fov_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/icvl/FOMA/src/fov/msg/FomaLocation.msg" NAME_WE)
add_dependencies(fov_generate_messages_py _fov_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/icvl/FOMA/src/fov/srv/Check.srv" NAME_WE)
add_dependencies(fov_generate_messages_py _fov_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/icvl/FOMA/src/fov/srv/Light.srv" NAME_WE)
add_dependencies(fov_generate_messages_py _fov_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/icvl/FOMA/src/fov/srv/Coordinate.srv" NAME_WE)
add_dependencies(fov_generate_messages_py _fov_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fov_genpy)
add_dependencies(fov_genpy fov_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fov_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fov)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fov
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(fov_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(fov_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fov)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fov
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(fov_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(fov_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fov)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fov
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(fov_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(fov_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fov)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fov
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(fov_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(fov_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fov)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fov\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fov
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(fov_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(fov_generate_messages_py geometry_msgs_generate_messages_py)
endif()
