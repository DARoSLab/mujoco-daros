include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/controllers)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/state_machine)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/robots)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/observers)

include_directories(SYSTEM "../../third-parties/qpOASES/include")

include_directories("/usr/local/include/lcm/") # lcm includes
include_directories("/usr/local/include/eigen3")

link_directories(/usr/local/lib)

add_subdirectory(observers)
add_subdirectory(controllers)
add_subdirectory(state_machine)
add_subdirectory(robots)

set(headers
  PatSystem.hpp
  PatSimulationBridge.hpp
  PatHardwareBridge.hpp
  PatHardwareTestBridge.hpp)

set(sources
  PatSystem.cpp
  PatSimulationBridge.cpp
  PatHardwareBridge.cpp
  PatHardwareTestBridge.cpp
  PatLCMCommunicator.cpp
  PatMocapManager.cpp
  )

add_library(patroclus_system SHARED ${headers} ${sources})
target_link_libraries(patroclus_system
    daros-utils daros-dynamics daros-dynamics_simulator daros-estimators
    lord_imu
    patroclus-robots patroclus-state_machine
    patroclus-controllers
  dynacore_param_handler ${realsense2_LIBRARY})
if(MOCAP_BUILD)
  target_link_libraries(patroclus_system owlsock)
endif()
if(CMAKE_SYSTEM_NAME MATCHES Linux)
    target_link_libraries(patroclus_system libvnc rs-rt)
endif()

if(IPOPT_ENABLE)
  add_subdirectory(controllers/RPC)
  target_link_libraries(patroclus_system rs-RPC)
endif()

## Executable File
add_executable(run_patroclus main.cpp)
target_link_libraries(run_patroclus patroclus_system)
