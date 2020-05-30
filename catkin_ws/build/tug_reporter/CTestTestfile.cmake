# CMake generated Testfile for 
# Source directory: /home/anna/catkin_ws/src/model_based_diagnosis/tug_examination/tug_reporter
# Build directory: /home/anna/catkin_ws/build/tug_reporter
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_tug_reporter_roslint_package "/home/anna/catkin_ws/build/tug_reporter/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/home/anna/catkin_ws/build/tug_reporter/test_results/tug_reporter/roslint-tug_reporter.xml" "--working-dir" "/home/anna/catkin_ws/build/tug_reporter" "--return-code" "/opt/ros/kinetic/share/roslint/cmake/../../../lib/roslint/test_wrapper /home/anna/catkin_ws/build/tug_reporter/test_results/tug_reporter/roslint-tug_reporter.xml make roslint_tug_reporter")
subdirs(gtest)
