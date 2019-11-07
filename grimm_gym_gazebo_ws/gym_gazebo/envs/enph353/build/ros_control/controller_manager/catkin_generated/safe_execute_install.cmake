execute_process(COMMAND "/home/fizzer/enph353-team-grimm/grimm_gym_gazebo_ws/gym_gazebo/envs/enph353/build/ros_control/controller_manager/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/fizzer/enph353-team-grimm/grimm_gym_gazebo_ws/gym_gazebo/envs/enph353/build/ros_control/controller_manager/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
