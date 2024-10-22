file(GLOB_RECURSE sources
  ${CMAKE_CURRENT_SOURCE_DIR}/convexMPC_Biped/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/auxillary/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/wbc_ctrl/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/PatRL/*.cpp)

add_library(patroclus-controllers SHARED ${sources})
if(RMPFLOW_BUILD)
target_link_libraries(patroclus-controllers rs-utils patroclus-robots
  rs-dynamics rs-WBC rs-RMPFlow ${ipopt_cheetah} qpOASES rs-ctrl_utils JCQP)
else()
target_link_libraries(patroclus-controllers rs-utils patroclus-robots
  rs-dynamics rs-WBC ${ipopt_cheetah} qpOASES rs-ctrl_utils JCQP)

endif()
