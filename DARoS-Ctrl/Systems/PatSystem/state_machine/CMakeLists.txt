file(GLOB_RECURSE sources "*.cpp")
add_library(patroclus-state_machine SHARED ${sources})
target_link_libraries(patroclus-state_machine rs-utils patroclus-robots
  patroclus-controllers patroclus-observers rs-rt)

if(IPOPT_ENABLE)
  target_link_libraries(patroclus-state_machine rs-RPC-patroclus)
endif()
