include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/StateMachine)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Robot)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Observer)

# add_library(ctrl_test SHARED 
#             PrestoeMJSimulationBridge.cpp
#             CtrlState/StateMachineCtrl.cpp
#             CtrlState/JPosCtrlState.cpp)

add_library(prestoe_ctrl_lib STATIC 
                      PrestoeSystem.cpp
                      StateMachine/StateMachineCtrl.cpp
                      StateMachine/JPosCtrlState.cpp)

target_link_libraries(prestoe_ctrl_lib daros-utils daros-common)

add_executable(run_prestoe main.cpp PrestoeMJSimulationBridge.cpp)
target_link_libraries(run_prestoe mujoco glfw prestoe_ctrl_lib)
 
