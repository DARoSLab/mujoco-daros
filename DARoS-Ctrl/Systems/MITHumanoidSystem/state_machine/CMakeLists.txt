#file(GLOB_RECURSE sources "*.cpp")
set(sources 
  ControlFSM_Tello.cpp
  Tello_State_JointPD.cpp
  Tello_State_BalanceStand.cpp
  Tello_State_Parkour.cpp
  Tello_State_Passive.cpp
  Tello_State.cpp
  SafetyChecker_Tello.cpp
  Tello_State_StandCasadi.cpp
  Tello_State_WalkCasadi.cpp
)

add_library(tello-state_machine SHARED ${sources})
target_link_libraries(tello-state_machine 
  daros-utils tello-robots
  tello-observers
  tello-controllers) 
