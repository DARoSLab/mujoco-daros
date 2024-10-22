#FILE(GLOB_RECURSE tellostand_sources "PrestoeStandCtrl/*.cpp")
#FILE(GLOB_RECURSE tellolocomotion_sources "PrestoeLocomotionCtrl/*.cpp")

#add_library(tello_WBC_Ctrl SHARED 
  #${tellostand_sources}
  #${tellolocomotion_sources}
  #WBC_Ctrl.cpp)

#target_link_libraries(tello_WBC_Ctrl daros-utils lcm daros-WBC
  #dynacore_param_handler daros-ctrl_utils)

