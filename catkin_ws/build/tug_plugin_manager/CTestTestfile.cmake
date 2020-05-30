# CMake generated Testfile for 
# Source directory: /home/anna/catkin_ws/src/utils/tug_plugin_manager
# Build directory: /home/anna/catkin_ws/build/tug_plugin_manager
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_tug_plugin_manager_roslint_package "/home/anna/catkin_ws/build/tug_plugin_manager/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/home/anna/catkin_ws/build/tug_plugin_manager/test_results/tug_plugin_manager/roslint-tug_plugin_manager.xml" "--working-dir" "/home/anna/catkin_ws/build/tug_plugin_manager" "--return-code" "/opt/ros/kinetic/share/roslint/cmake/../../../lib/roslint/test_wrapper /home/anna/catkin_ws/build/tug_plugin_manager/test_results/tug_plugin_manager/roslint-tug_plugin_manager.xml make roslint_tug_plugin_manager")
subdirs(gtest)
