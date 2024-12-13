find_package(lcm REQUIRED)
include_directories("/usr/local/include/lcm/")   # lcm includes

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/StateMachine)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Controller)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Robot)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Observer)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/LCM-Types/cpp)

# add_library(ctrl_test SHARED 
#             PrestoeBipedMJSimulationBridge.cpp
#             CtrlState/StateMachineCtrl.cpp
#             CtrlState/JPosCtrlState.cpp)

file(GLOB_RECURSE WBC_SRC Controller/WBC_Prestoe/*.cpp)

add_library(prestoebiped_ctrl_lib STATIC 
                      PrestoeBipedSystem.cpp
                      StateMachine/StateMachineCtrl.cpp
                      StateMachine/JPosCtrlState.cpp
                      StateMachine/BalanceStandState.cpp
                      ${WBC_SRC}
                      )

target_link_libraries(prestoebiped_ctrl_lib daros-utils daros-ctrl-utils lcm)

add_executable(run_prestoebiped main.cpp PrestoeBipedMJSimulationBridge.cpp)
target_link_libraries(run_prestoebiped mujoco glfw prestoebiped_ctrl_lib)
 
