# CMake generated Testfile for 
# Source directory: /home/qh/Desktop/mujoco-daros/test/sample
# Build directory: /home/qh/Desktop/mujoco-daros/test/sample
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(compile_test_setup "/usr/bin/bash" "/home/qh/Desktop/mujoco-daros/cmake/setup_test_dir.sh" "/home/qh/Desktop/mujoco-daros/test/sample/compile_test")
set_tests_properties(compile_test_setup PROPERTIES  FIXTURES_SETUP "compile_test_fixture" _BACKTRACE_TRIPLES "/home/qh/Desktop/mujoco-daros/cmake/ShellTests.cmake;20;add_test;/home/qh/Desktop/mujoco-daros/test/sample/CMakeLists.txt;18;add_mujoco_shell_test;/home/qh/Desktop/mujoco-daros/test/sample/CMakeLists.txt;0;")
add_test(compile_test_cleanup "/usr/bin/bash" "/home/qh/Desktop/mujoco-daros/cmake/cleanup_test_dir.sh" "/home/qh/Desktop/mujoco-daros/test/sample/compile_test")
set_tests_properties(compile_test_cleanup PROPERTIES  FIXTURES_CLEANUP "compile_test_fixture" _BACKTRACE_TRIPLES "/home/qh/Desktop/mujoco-daros/cmake/ShellTests.cmake;24;add_test;/home/qh/Desktop/mujoco-daros/test/sample/CMakeLists.txt;18;add_mujoco_shell_test;/home/qh/Desktop/mujoco-daros/test/sample/CMakeLists.txt;0;")
add_test(compile_test "/usr/bin/bash" "/home/qh/Desktop/mujoco-daros/test/sample/compile_test.sh")
set_tests_properties(compile_test PROPERTIES  ENVIRONMENT "CMAKE_SOURCE_DIR=/home/qh/Desktop/mujoco-daros;TARGET_BINARY=/home/qh/Desktop/mujoco-daros/bin/compile;TEST_TMPDIR=/home/qh/Desktop/mujoco-daros/test/sample/compile_test" FIXTURES_REQUIRED "compile_test_fixture" WORKING_DIRECTORY "/home/qh/Desktop/mujoco-daros/bin" _BACKTRACE_TRIPLES "/home/qh/Desktop/mujoco-daros/cmake/ShellTests.cmake;29;add_test;/home/qh/Desktop/mujoco-daros/test/sample/CMakeLists.txt;18;add_mujoco_shell_test;/home/qh/Desktop/mujoco-daros/test/sample/CMakeLists.txt;0;")
add_test(testspeed_test_setup "/usr/bin/bash" "/home/qh/Desktop/mujoco-daros/cmake/setup_test_dir.sh" "/home/qh/Desktop/mujoco-daros/test/sample/testspeed_test")
set_tests_properties(testspeed_test_setup PROPERTIES  FIXTURES_SETUP "testspeed_test_fixture" _BACKTRACE_TRIPLES "/home/qh/Desktop/mujoco-daros/cmake/ShellTests.cmake;20;add_test;/home/qh/Desktop/mujoco-daros/test/sample/CMakeLists.txt;19;add_mujoco_shell_test;/home/qh/Desktop/mujoco-daros/test/sample/CMakeLists.txt;0;")
add_test(testspeed_test_cleanup "/usr/bin/bash" "/home/qh/Desktop/mujoco-daros/cmake/cleanup_test_dir.sh" "/home/qh/Desktop/mujoco-daros/test/sample/testspeed_test")
set_tests_properties(testspeed_test_cleanup PROPERTIES  FIXTURES_CLEANUP "testspeed_test_fixture" _BACKTRACE_TRIPLES "/home/qh/Desktop/mujoco-daros/cmake/ShellTests.cmake;24;add_test;/home/qh/Desktop/mujoco-daros/test/sample/CMakeLists.txt;19;add_mujoco_shell_test;/home/qh/Desktop/mujoco-daros/test/sample/CMakeLists.txt;0;")
add_test(testspeed_test "/usr/bin/bash" "/home/qh/Desktop/mujoco-daros/test/sample/testspeed_test.sh")
set_tests_properties(testspeed_test PROPERTIES  ENVIRONMENT "CMAKE_SOURCE_DIR=/home/qh/Desktop/mujoco-daros;TARGET_BINARY=/home/qh/Desktop/mujoco-daros/bin/testspeed;TEST_TMPDIR=/home/qh/Desktop/mujoco-daros/test/sample/testspeed_test" FIXTURES_REQUIRED "testspeed_test_fixture" WORKING_DIRECTORY "/home/qh/Desktop/mujoco-daros/bin" _BACKTRACE_TRIPLES "/home/qh/Desktop/mujoco-daros/cmake/ShellTests.cmake;29;add_test;/home/qh/Desktop/mujoco-daros/test/sample/CMakeLists.txt;19;add_mujoco_shell_test;/home/qh/Desktop/mujoco-daros/test/sample/CMakeLists.txt;0;")
