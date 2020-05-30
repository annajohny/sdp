# CMake generated Testfile for 
# Source directory: /home/anna/catkin_ws/src/utils/tug_testing
# Build directory: /home/anna/catkin_ws/build/tug_testing
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_tug_testing_roslint_package "/home/anna/catkin_ws/build/tug_testing/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/home/anna/catkin_ws/build/tug_testing/test_results/tug_testing/roslint-tug_testing.xml" "--working-dir" "/home/anna/catkin_ws/build/tug_testing" "--return-code" "/opt/ros/kinetic/share/roslint/cmake/../../../lib/roslint/test_wrapper /home/anna/catkin_ws/build/tug_testing/test_results/tug_testing/roslint-tug_testing.xml make roslint_tug_testing")
subdirs(gtest)
