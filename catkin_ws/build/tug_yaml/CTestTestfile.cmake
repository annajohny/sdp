# CMake generated Testfile for 
# Source directory: /home/anna/catkin_ws/src/utils/tug_yaml
# Build directory: /home/anna/catkin_ws/build/tug_yaml
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_tug_yaml_roslint_package "/home/anna/catkin_ws/build/tug_yaml/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/home/anna/catkin_ws/build/tug_yaml/test_results/tug_yaml/roslint-tug_yaml.xml" "--working-dir" "/home/anna/catkin_ws/build/tug_yaml" "--return-code" "/opt/ros/kinetic/share/roslint/cmake/../../../lib/roslint/test_wrapper /home/anna/catkin_ws/build/tug_yaml/test_results/tug_yaml/roslint-tug_yaml.xml make roslint_tug_yaml")
subdirs(gtest)
