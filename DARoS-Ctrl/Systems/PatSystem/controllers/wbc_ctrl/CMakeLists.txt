FILE(GLOB_RECURSE locomotion_headers "LocomotionCtrl/*.hpp")
FILE(GLOB_RECURSE locomotion_sources "LocomotionCtrl/*.cpp")

add_library(quadruped_WBC_Ctrl SHARED 
    ${locomotion_headers} ${locomotion_sources}
    WBC_Ctrl.cpp)

target_link_libraries(quadruped_WBC_Ctrl rs-utils lcm WBC
  dynacore_param_handler)

