find_package(lcm REQUIRED)
include_directories("/usr/local/include/lcm/")   # lcm includes

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/StateMachine)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Controller)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Robot)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Observer)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/LCM-Types/cpp)

# add_library(ctrl_test SHARED 
#             PrestoeMJSimulationBridge.cpp
#             CtrlState/StateMachineCtrl.cpp
#             CtrlState/JPosCtrlState.cpp)

file(GLOB_RECURSE WBC_SRC Controller/WBC_Prestoe/*.cpp)

add_library(prestoe_ctrl_lib STATIC 
                      PrestoeSystem.cpp
                      StateMachine/StateMachineCtrl.cpp
                      StateMachine/JPosCtrlState.cpp
                      StateMachine/BalanceStandState.cpp
                      ${WBC_SRC}
                      )

target_link_libraries(prestoe_ctrl_lib daros-utils daros-ctrl-utils lcm)

add_executable(run_prestoe main.cpp PrestoeMJSimulationBridge.cpp)
target_link_libraries(run_prestoe mujoco glfw prestoe_ctrl_lib)
 
