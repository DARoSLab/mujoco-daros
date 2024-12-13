# CMake generated Testfile for 
# Source directory: /home/qh/Desktop/mujoco-daros/test/plugin/actuator
# Build directory: /home/qh/Desktop/mujoco-daros/test/plugin/actuator
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(PidTest.PGain "/home/qh/Desktop/mujoco-daros/bin/pid_test" "--gtest_filter=PidTest.PGain")
set_tests_properties(PidTest.PGain PROPERTIES  ENVIRONMENT "MUJOCO_PLUGIN_DIR=/home/qh/Desktop/mujoco-daros/lib" WORKING_DIRECTORY "/home/qh/Desktop/mujoco-daros/test" _BACKTRACE_TRIPLES "/usr/share/cmake-3.22/Modules/GoogleTest.cmake;400;add_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;37;gtest_add_tests;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;15;mujoco_test;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;0;")
add_test(PidTest.PGainWithFilterExact "/home/qh/Desktop/mujoco-daros/bin/pid_test" "--gtest_filter=PidTest.PGainWithFilterExact")
set_tests_properties(PidTest.PGainWithFilterExact PROPERTIES  ENVIRONMENT "MUJOCO_PLUGIN_DIR=/home/qh/Desktop/mujoco-daros/lib" WORKING_DIRECTORY "/home/qh/Desktop/mujoco-daros/test" _BACKTRACE_TRIPLES "/usr/share/cmake-3.22/Modules/GoogleTest.cmake;400;add_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;37;gtest_add_tests;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;15;mujoco_test;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;0;")
add_test(PidTest.SlewMaxRate "/home/qh/Desktop/mujoco-daros/bin/pid_test" "--gtest_filter=PidTest.SlewMaxRate")
set_tests_properties(PidTest.SlewMaxRate PROPERTIES  ENVIRONMENT "MUJOCO_PLUGIN_DIR=/home/qh/Desktop/mujoco-daros/lib" WORKING_DIRECTORY "/home/qh/Desktop/mujoco-daros/test" _BACKTRACE_TRIPLES "/usr/share/cmake-3.22/Modules/GoogleTest.cmake;400;add_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;37;gtest_add_tests;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;15;mujoco_test;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;0;")
add_test(PidTest.IntegratedVelocitySlewMaxRate "/home/qh/Desktop/mujoco-daros/bin/pid_test" "--gtest_filter=PidTest.IntegratedVelocitySlewMaxRate")
set_tests_properties(PidTest.IntegratedVelocitySlewMaxRate PROPERTIES  ENVIRONMENT "MUJOCO_PLUGIN_DIR=/home/qh/Desktop/mujoco-daros/lib" WORKING_DIRECTORY "/home/qh/Desktop/mujoco-daros/test" _BACKTRACE_TRIPLES "/usr/share/cmake-3.22/Modules/GoogleTest.cmake;400;add_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;37;gtest_add_tests;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;15;mujoco_test;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;0;")
add_test(PidTest.SlewMaxRateUsesFirstCtrl "/home/qh/Desktop/mujoco-daros/bin/pid_test" "--gtest_filter=PidTest.SlewMaxRateUsesFirstCtrl")
set_tests_properties(PidTest.SlewMaxRateUsesFirstCtrl PROPERTIES  ENVIRONMENT "MUJOCO_PLUGIN_DIR=/home/qh/Desktop/mujoco-daros/lib" WORKING_DIRECTORY "/home/qh/Desktop/mujoco-daros/test" _BACKTRACE_TRIPLES "/usr/share/cmake-3.22/Modules/GoogleTest.cmake;400;add_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;37;gtest_add_tests;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;15;mujoco_test;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;0;")
add_test(PidTest.ITerm "/home/qh/Desktop/mujoco-daros/bin/pid_test" "--gtest_filter=PidTest.ITerm")
set_tests_properties(PidTest.ITerm PROPERTIES  ENVIRONMENT "MUJOCO_PLUGIN_DIR=/home/qh/Desktop/mujoco-daros/lib" WORKING_DIRECTORY "/home/qh/Desktop/mujoco-daros/test" _BACKTRACE_TRIPLES "/usr/share/cmake-3.22/Modules/GoogleTest.cmake;400;add_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;37;gtest_add_tests;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;15;mujoco_test;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;0;")
add_test(PidTest.FiniteDifferencing "/home/qh/Desktop/mujoco-daros/bin/pid_test" "--gtest_filter=PidTest.FiniteDifferencing")
set_tests_properties(PidTest.FiniteDifferencing PROPERTIES  ENVIRONMENT "MUJOCO_PLUGIN_DIR=/home/qh/Desktop/mujoco-daros/lib" WORKING_DIRECTORY "/home/qh/Desktop/mujoco-daros/test" _BACKTRACE_TRIPLES "/usr/share/cmake-3.22/Modules/GoogleTest.cmake;400;add_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;37;gtest_add_tests;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;15;mujoco_test;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;0;")
add_test(PidTest.CtrlClamp "/home/qh/Desktop/mujoco-daros/bin/pid_test" "--gtest_filter=PidTest.CtrlClamp")
set_tests_properties(PidTest.CtrlClamp PROPERTIES  ENVIRONMENT "MUJOCO_PLUGIN_DIR=/home/qh/Desktop/mujoco-daros/lib" WORKING_DIRECTORY "/home/qh/Desktop/mujoco-daros/test" _BACKTRACE_TRIPLES "/usr/share/cmake-3.22/Modules/GoogleTest.cmake;400;add_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;37;gtest_add_tests;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;15;mujoco_test;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;0;")
add_test(PidTest.CopyData "/home/qh/Desktop/mujoco-daros/bin/pid_test" "--gtest_filter=PidTest.CopyData")
set_tests_properties(PidTest.CopyData PROPERTIES  ENVIRONMENT "MUJOCO_PLUGIN_DIR=/home/qh/Desktop/mujoco-daros/lib" WORKING_DIRECTORY "/home/qh/Desktop/mujoco-daros/test" _BACKTRACE_TRIPLES "/usr/share/cmake-3.22/Modules/GoogleTest.cmake;400;add_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;37;gtest_add_tests;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;15;mujoco_test;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;0;")
add_test(PidTest.MultipleActuatorsSamePlugin "/home/qh/Desktop/mujoco-daros/bin/pid_test" "--gtest_filter=PidTest.MultipleActuatorsSamePlugin")
set_tests_properties(PidTest.MultipleActuatorsSamePlugin PROPERTIES  ENVIRONMENT "MUJOCO_PLUGIN_DIR=/home/qh/Desktop/mujoco-daros/lib" WORKING_DIRECTORY "/home/qh/Desktop/mujoco-daros/test" _BACKTRACE_TRIPLES "/usr/share/cmake-3.22/Modules/GoogleTest.cmake;400;add_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;37;gtest_add_tests;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;15;mujoco_test;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;0;")
add_test(PidTest.InvalidClamp "/home/qh/Desktop/mujoco-daros/bin/pid_test" "--gtest_filter=PidTest.InvalidClamp")
set_tests_properties(PidTest.InvalidClamp PROPERTIES  ENVIRONMENT "MUJOCO_PLUGIN_DIR=/home/qh/Desktop/mujoco-daros/lib" WORKING_DIRECTORY "/home/qh/Desktop/mujoco-daros/test" _BACKTRACE_TRIPLES "/usr/share/cmake-3.22/Modules/GoogleTest.cmake;400;add_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;37;gtest_add_tests;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;15;mujoco_test;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;0;")
add_test(PidTest.InvalidSlew "/home/qh/Desktop/mujoco-daros/bin/pid_test" "--gtest_filter=PidTest.InvalidSlew")
set_tests_properties(PidTest.InvalidSlew PROPERTIES  ENVIRONMENT "MUJOCO_PLUGIN_DIR=/home/qh/Desktop/mujoco-daros/lib" WORKING_DIRECTORY "/home/qh/Desktop/mujoco-daros/test" _BACKTRACE_TRIPLES "/usr/share/cmake-3.22/Modules/GoogleTest.cmake;400;add_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;37;gtest_add_tests;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;15;mujoco_test;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;0;")
add_test(PidTest.WrongActdim "/home/qh/Desktop/mujoco-daros/bin/pid_test" "--gtest_filter=PidTest.WrongActdim")
set_tests_properties(PidTest.WrongActdim PROPERTIES  ENVIRONMENT "MUJOCO_PLUGIN_DIR=/home/qh/Desktop/mujoco-daros/lib" WORKING_DIRECTORY "/home/qh/Desktop/mujoco-daros/test" _BACKTRACE_TRIPLES "/usr/share/cmake-3.22/Modules/GoogleTest.cmake;400;add_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;37;gtest_add_tests;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;15;mujoco_test;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;0;")
add_test(PidTest.Keyframe "/home/qh/Desktop/mujoco-daros/bin/pid_test" "--gtest_filter=PidTest.Keyframe")
set_tests_properties(PidTest.Keyframe PROPERTIES  ENVIRONMENT "MUJOCO_PLUGIN_DIR=/home/qh/Desktop/mujoco-daros/lib" WORKING_DIRECTORY "/home/qh/Desktop/mujoco-daros/test" _BACKTRACE_TRIPLES "/usr/share/cmake-3.22/Modules/GoogleTest.cmake;400;add_test;/home/qh/Desktop/mujoco-daros/test/CMakeLists.txt;37;gtest_add_tests;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;15;mujoco_test;/home/qh/Desktop/mujoco-daros/test/plugin/actuator/CMakeLists.txt;0;")
