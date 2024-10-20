include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/CtrlState)

# add_library(ctrl_test SHARED 
#             PrestoeMJSimulationBridge.cpp
#             CtrlState/StateMachineCtrl.cpp
#             CtrlState/JPosCtrlState.cpp)

add_library(ctrl_test PrestoeMJSimulationBridge.cpp)
target_link_libraries(ctrl_test mujoco glfw)

add_executable(run_prestoe main.cpp)
target_link_libraries(run_prestoe ctrl_test)
 