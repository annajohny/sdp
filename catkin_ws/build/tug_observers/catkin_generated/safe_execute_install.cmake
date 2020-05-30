execute_process(COMMAND "/home/anna/catkin_ws/build/tug_observers/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/anna/catkin_ws/build/tug_observers/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
