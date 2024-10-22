file(GLOB_RECURSE sources
  #${CMAKE_CURRENT_SOURCE_DIR}/cMPC_Biped/*.cpp
  #${CMAKE_CURRENT_SOURCE_DIR}/MPC_Biped/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/MPC_Casadi/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/wbc_ctrl/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/auxillary/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/RL_MPC/*.cpp)


add_library(tello-controllers SHARED ${sources})
#target_link_libraries(tello-controllers daros-utils tello-robots daros-dynamics daros-WBC casadi loco-jump_utils) 
target_link_libraries(tello-controllers daros-utils tello-robots daros-dynamics daros-WBC casadi loco-jump_system) 

