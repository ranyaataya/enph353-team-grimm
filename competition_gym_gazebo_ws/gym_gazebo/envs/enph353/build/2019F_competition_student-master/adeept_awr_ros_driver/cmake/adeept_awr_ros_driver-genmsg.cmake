# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "adeept_awr_ros_driver: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iadeept_awr_ros_driver:/home/fizzer/enph353-team-grimm/competition_gym_gazebo_ws/gym_gazebo/envs/enph353/src/2019F_competition_student-master/adeept_awr_ros_driver/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(adeept_awr_ros_driver_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/fizzer/enph353-team-grimm/competition_gym_gazebo_ws/gym_gazebo/envs/enph353/src/2019F_competition_student-master/adeept_awr_ros_driver/msg/ArrayIR.msg" NAME_WE)
add_custom_target(_adeept_awr_ros_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "adeept_awr_ros_driver" "/home/fizzer/enph353-team-grimm/competition_gym_gazebo_ws/gym_gazebo/envs/enph353/src/2019F_competition_student-master/adeept_awr_ros_driver/msg/ArrayIR.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(adeept_awr_ros_driver
  "/home/fizzer/enph353-team-grimm/competition_gym_gazebo_ws/gym_gazebo/envs/enph353/src/2019F_competition_student-master/adeept_awr_ros_driver/msg/ArrayIR.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/adeept_awr_ros_driver
)

### Generating Services

### Generating Module File
_generate_module_cpp(adeept_awr_ros_driver
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/adeept_awr_ros_driver
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(adeept_awr_ros_driver_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(adeept_awr_ros_driver_generate_messages adeept_awr_ros_driver_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fizzer/enph353-team-grimm/competition_gym_gazebo_ws/gym_gazebo/envs/enph353/src/2019F_competition_student-master/adeept_awr_ros_driver/msg/ArrayIR.msg" NAME_WE)
add_dependencies(adeept_awr_ros_driver_generate_messages_cpp _adeept_awr_ros_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(adeept_awr_ros_driver_gencpp)
add_dependencies(adeept_awr_ros_driver_gencpp adeept_awr_ros_driver_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS adeept_awr_ros_driver_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(adeept_awr_ros_driver
  "/home/fizzer/enph353-team-grimm/competition_gym_gazebo_ws/gym_gazebo/envs/enph353/src/2019F_competition_student-master/adeept_awr_ros_driver/msg/ArrayIR.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/adeept_awr_ros_driver
)

### Generating Services

### Generating Module File
_generate_module_eus(adeept_awr_ros_driver
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/adeept_awr_ros_driver
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(adeept_awr_ros_driver_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(adeept_awr_ros_driver_generate_messages adeept_awr_ros_driver_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fizzer/enph353-team-grimm/competition_gym_gazebo_ws/gym_gazebo/envs/enph353/src/2019F_competition_student-master/adeept_awr_ros_driver/msg/ArrayIR.msg" NAME_WE)
add_dependencies(adeept_awr_ros_driver_generate_messages_eus _adeept_awr_ros_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(adeept_awr_ros_driver_geneus)
add_dependencies(adeept_awr_ros_driver_geneus adeept_awr_ros_driver_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS adeept_awr_ros_driver_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(adeept_awr_ros_driver
  "/home/fizzer/enph353-team-grimm/competition_gym_gazebo_ws/gym_gazebo/envs/enph353/src/2019F_competition_student-master/adeept_awr_ros_driver/msg/ArrayIR.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/adeept_awr_ros_driver
)

### Generating Services

### Generating Module File
_generate_module_lisp(adeept_awr_ros_driver
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/adeept_awr_ros_driver
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(adeept_awr_ros_driver_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(adeept_awr_ros_driver_generate_messages adeept_awr_ros_driver_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fizzer/enph353-team-grimm/competition_gym_gazebo_ws/gym_gazebo/envs/enph353/src/2019F_competition_student-master/adeept_awr_ros_driver/msg/ArrayIR.msg" NAME_WE)
add_dependencies(adeept_awr_ros_driver_generate_messages_lisp _adeept_awr_ros_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(adeept_awr_ros_driver_genlisp)
add_dependencies(adeept_awr_ros_driver_genlisp adeept_awr_ros_driver_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS adeept_awr_ros_driver_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(adeept_awr_ros_driver
  "/home/fizzer/enph353-team-grimm/competition_gym_gazebo_ws/gym_gazebo/envs/enph353/src/2019F_competition_student-master/adeept_awr_ros_driver/msg/ArrayIR.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/adeept_awr_ros_driver
)

### Generating Services

### Generating Module File
_generate_module_nodejs(adeept_awr_ros_driver
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/adeept_awr_ros_driver
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(adeept_awr_ros_driver_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(adeept_awr_ros_driver_generate_messages adeept_awr_ros_driver_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fizzer/enph353-team-grimm/competition_gym_gazebo_ws/gym_gazebo/envs/enph353/src/2019F_competition_student-master/adeept_awr_ros_driver/msg/ArrayIR.msg" NAME_WE)
add_dependencies(adeept_awr_ros_driver_generate_messages_nodejs _adeept_awr_ros_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(adeept_awr_ros_driver_gennodejs)
add_dependencies(adeept_awr_ros_driver_gennodejs adeept_awr_ros_driver_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS adeept_awr_ros_driver_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(adeept_awr_ros_driver
  "/home/fizzer/enph353-team-grimm/competition_gym_gazebo_ws/gym_gazebo/envs/enph353/src/2019F_competition_student-master/adeept_awr_ros_driver/msg/ArrayIR.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/adeept_awr_ros_driver
)

### Generating Services

### Generating Module File
_generate_module_py(adeept_awr_ros_driver
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/adeept_awr_ros_driver
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(adeept_awr_ros_driver_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(adeept_awr_ros_driver_generate_messages adeept_awr_ros_driver_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fizzer/enph353-team-grimm/competition_gym_gazebo_ws/gym_gazebo/envs/enph353/src/2019F_competition_student-master/adeept_awr_ros_driver/msg/ArrayIR.msg" NAME_WE)
add_dependencies(adeept_awr_ros_driver_generate_messages_py _adeept_awr_ros_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(adeept_awr_ros_driver_genpy)
add_dependencies(adeept_awr_ros_driver_genpy adeept_awr_ros_driver_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS adeept_awr_ros_driver_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/adeept_awr_ros_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/adeept_awr_ros_driver
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(adeept_awr_ros_driver_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/adeept_awr_ros_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/adeept_awr_ros_driver
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(adeept_awr_ros_driver_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/adeept_awr_ros_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/adeept_awr_ros_driver
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(adeept_awr_ros_driver_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/adeept_awr_ros_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/adeept_awr_ros_driver
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(adeept_awr_ros_driver_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/adeept_awr_ros_driver)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/adeept_awr_ros_driver\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/adeept_awr_ros_driver
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(adeept_awr_ros_driver_generate_messages_py std_msgs_generate_messages_py)
endif()