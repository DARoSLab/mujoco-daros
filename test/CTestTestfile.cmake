# CMake generated Testfile for 
# Source directory: /home/qh/Desktop/mujoco-daros/test
# Build directory: /home/qh/Desktop/mujoco-daros/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(MujocoTestTest.MjUserWarningFailsTest "/home/qh/Desktop/mujoco-daros/bin/fixture_test" "--gtest_filter=MujocoTestTest.MjUserWarningFailsTest")
set_tests_properties(MujocoTestTest.MjUserWarningFailsTest PROPERTIES  WORKING_DIRECTORY "/home/qh/Desktop/mujoco-daros/test" _BACKTRACE_TRIPLES "/usr/share/cmake-3.22/Modules/GoogleTest.cmake;400;add_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;37;gtest_add_tests;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;69;mujoco_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;0;")
add_test(MujocoTestTest.MjUserErrorFailsTest "/home/qh/Desktop/mujoco-daros/bin/fixture_test" "--gtest_filter=MujocoTestTest.MjUserErrorFailsTest")
set_tests_properties(MujocoTestTest.MjUserErrorFailsTest PROPERTIES  WORKING_DIRECTORY "/home/qh/Desktop/mujoco-daros/test" _BACKTRACE_TRIPLES "/usr/share/cmake-3.22/Modules/GoogleTest.cmake;400;add_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;37;gtest_add_tests;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;69;mujoco_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;0;")
add_test(MujocoErrorTestGuardTest.NestedErrorGuards "/home/qh/Desktop/mujoco-daros/bin/fixture_test" "--gtest_filter=MujocoErrorTestGuardTest.NestedErrorGuards")
set_tests_properties(MujocoErrorTestGuardTest.NestedErrorGuards PROPERTIES  WORKING_DIRECTORY "/home/qh/Desktop/mujoco-daros/test" _BACKTRACE_TRIPLES "/usr/share/cmake-3.22/Modules/GoogleTest.cmake;400;add_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;37;gtest_add_tests;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;69;mujoco_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;0;")
add_test(MujocoTestTest.MockFilesystemTest "/home/qh/Desktop/mujoco-daros/bin/fixture_test" "--gtest_filter=MujocoTestTest.MockFilesystemTest")
set_tests_properties(MujocoTestTest.MockFilesystemTest PROPERTIES  WORKING_DIRECTORY "/home/qh/Desktop/mujoco-daros/test" _BACKTRACE_TRIPLES "/usr/share/cmake-3.22/Modules/GoogleTest.cmake;400;add_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;37;gtest_add_tests;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;69;mujoco_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;0;")
add_test(HeaderTest.IntsHave4Bytes "/home/qh/Desktop/mujoco-daros/bin/header_test" "--gtest_filter=HeaderTest.IntsHave4Bytes")
set_tests_properties(HeaderTest.IntsHave4Bytes PROPERTIES  WORKING_DIRECTORY "/home/qh/Desktop/mujoco-daros/test" _BACKTRACE_TRIPLES "/usr/share/cmake-3.22/Modules/GoogleTest.cmake;400;add_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;37;gtest_add_tests;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;72;mujoco_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;0;")
add_test(HeaderTest.IntsHaveAtLeast31Bits "/home/qh/Desktop/mujoco-daros/bin/header_test" "--gtest_filter=HeaderTest.IntsHaveAtLeast31Bits")
set_tests_properties(HeaderTest.IntsHaveAtLeast31Bits PROPERTIES  WORKING_DIRECTORY "/home/qh/Desktop/mujoco-daros/test" _BACKTRACE_TRIPLES "/usr/share/cmake-3.22/Modules/GoogleTest.cmake;400;add_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;37;gtest_add_tests;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;72;mujoco_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;0;")
add_test(HeaderTest.EnumsAreInts "/home/qh/Desktop/mujoco-daros/bin/header_test" "--gtest_filter=HeaderTest.EnumsAreInts")
set_tests_properties(HeaderTest.EnumsAreInts PROPERTIES  WORKING_DIRECTORY "/home/qh/Desktop/mujoco-daros/test" _BACKTRACE_TRIPLES "/usr/share/cmake-3.22/Modules/GoogleTest.cmake;400;add_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;37;gtest_add_tests;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;72;mujoco_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;0;")
add_test(PipelineTest.SparseDenseEquivalent "/home/qh/Desktop/mujoco-daros/bin/pipeline_test" "--gtest_filter=PipelineTest.SparseDenseEquivalent")
set_tests_properties(PipelineTest.SparseDenseEquivalent PROPERTIES  WORKING_DIRECTORY "/home/qh/Desktop/mujoco-daros/test" _BACKTRACE_TRIPLES "/usr/share/cmake-3.22/Modules/GoogleTest.cmake;400;add_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;37;gtest_add_tests;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;75;mujoco_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;0;")
add_test(PipelineTest.DeterministicNoWarmstart "/home/qh/Desktop/mujoco-daros/bin/pipeline_test" "--gtest_filter=PipelineTest.DeterministicNoWarmstart")
set_tests_properties(PipelineTest.DeterministicNoWarmstart PROPERTIES  WORKING_DIRECTORY "/home/qh/Desktop/mujoco-daros/test" _BACKTRACE_TRIPLES "/usr/share/cmake-3.22/Modules/GoogleTest.cmake;400;add_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;37;gtest_add_tests;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;75;mujoco_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;0;")
subdirs("benchmark")
subdirs("engine")
subdirs("sample")
subdirs("user")
subdirs("xml")
subdirs("plugin/elasticity")
subdirs("plugin/actuator")
