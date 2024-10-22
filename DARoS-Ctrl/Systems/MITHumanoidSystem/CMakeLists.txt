include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/controllers)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/state_machine)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/observers)

include_directories(${CMAKE_SOURCE_DIR}/3rd-parties)
include_directories(SYSTEM "../../third-parties/qpOASES/include")

include_directories("/usr/local/include/lcm/") # lcm includes
include_directories("/usr/local/include/eigen3")
include_directories("./")

# casadi stuff
find_package(casadi REQUIRED) 
include_directories("/usr/local/include/casadi/")
find_package(Torch REQUIRED) 


link_directories(/usr/local/lib)

add_subdirectory(controllers)
add_subdirectory(state_machine)
add_subdirectory(robots)
add_subdirectory(observers)

set(sources
  TelloSystem.cpp
  TelloSimulationBridge.cpp
  TelloLCMCommunicator.cpp)

add_library(tello_system SHARED ${sources})
target_link_libraries(tello_system 
  daros-utils daros-dynamics daros-dynamics_simulator 
  tello-robots 
  tello-controllers 
  tello-observers
  tello-state_machine 
  daros-param_handler
  casadi )

## Executable File
add_executable(run_tello main.cpp)
target_link_libraries(run_tello tello_system)

