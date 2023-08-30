# CMake generated Testfile for 
# Source directory: /home/taizun/Desktop/legged_control_zidong/src/legged_wbc
# Build directory: /home/taizun/Desktop/legged_control_zidong/src/legged_wbc/cmake-build-debug
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_legged_wbc_gtest_legged_wbc_test "/home/taizun/Desktop/legged_control_zidong/src/legged_wbc/cmake-build-debug/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/taizun/Desktop/legged_control_zidong/src/legged_wbc/cmake-build-debug/test_results/legged_wbc/gtest-legged_wbc_test.xml" "--return-code" "/home/taizun/Desktop/legged_control_zidong/src/legged_wbc/cmake-build-debug/devel/lib/legged_wbc/legged_wbc_test --gtest_output=xml:/home/taizun/Desktop/legged_control_zidong/src/legged_wbc/cmake-build-debug/test_results/legged_wbc/gtest-legged_wbc_test.xml")
set_tests_properties(_ctest_legged_wbc_gtest_legged_wbc_test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/taizun/Desktop/legged_control_zidong/src/legged_wbc/CMakeLists.txt;94;catkin_add_gtest;/home/taizun/Desktop/legged_control_zidong/src/legged_wbc/CMakeLists.txt;0;")
subdirs("gtest")
