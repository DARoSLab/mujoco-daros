#FILE(GLOB_RECURSE prestoestand_sources "PrestoeStandCtrl/*.cpp")
#FILE(GLOB_RECURSE prestoelocomotion_sources "PrestoeLocomotionCtrl/*.cpp")

#add_library(prestoe_WBC_Ctrl SHARED 
  #${prestoestand_sources}
  #${prestoelocomotion_sources}
  #WBC_Ctrl.cpp)

#target_link_libraries(prestoe_WBC_Ctrl daros-utils lcm daros-WBC
  #dynacore_param_handler daros-ctrl_utils)

