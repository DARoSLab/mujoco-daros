include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/CtrlState)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Observer)

# add_library(ctrl_test SHARED 
#             PrestoeMJSimulationBridge.cpp
#             CtrlState/StateMachineCtrl.cpp
#             CtrlState/JPosCtrlState.cpp)

add_library(ctrl_test STATIC 
                      PrestoeMJSimulationBridge.cpp 
                      PrestoeSystem.cpp
                      CtrlState/StateMachineCtrl.cpp
                      CtrlState/JPosCtrlState.cpp)

target_link_libraries(ctrl_test mujoco glfw daros-utils)

add_executable(run_prestoe main.cpp)
target_link_libraries(run_prestoe ctrl_test)
 
