file(GLOB_RECURSE sources "*.cpp")

#message(${sources})
add_library(daros-dynamics_simulator SHARED ${sources})
target_link_libraries(daros-dynamics_simulator 
  daros-utils lcm daros-dynamics
  daros-param_handler)

