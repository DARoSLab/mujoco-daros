include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/StateMachine)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Robot)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Observer)

add_library(miniarm_ctrl_lib STATIC 
                      MiniArmSystem.cpp
                      StateMachine/StateMachineCtrl.cpp
                      StateMachine/JPosCtrlState.cpp)

target_link_libraries(miniarm_ctrl_lib daros-utils daros-ctrl-utils)

add_executable(run_miniarm main.cpp MiniArmMJSimulationBridge.cpp)
target_link_libraries(run_miniarm mujoco glfw miniarm_ctrl_lib)
 
