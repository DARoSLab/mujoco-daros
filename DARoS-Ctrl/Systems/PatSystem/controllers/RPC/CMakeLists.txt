if(IPOPT_ENABLE)
  FILE(GLOB_RECURSE rpc_headers ./*.h)
  FILE(GLOB_RECURSE rpc_sources ./*.cpp)

  add_library(rs-RPC-patroclus SHARED ${rpc_sources})
  target_link_libraries(rs-RPC-patroclus ${TORCH_LIBRARIES} ${ipopt_cheetah} lcm
    patroclus-controllers)
endif()
