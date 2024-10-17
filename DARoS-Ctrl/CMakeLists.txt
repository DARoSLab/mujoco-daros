set(THIS_COM "${PROJECT_SOURCE_DIR}/DARoS-Ctrl" )
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_BINARY_DIR}/Configuration.h)

add_executable(daros-ctrl main.cpp)
target_link_libraries(daros-ctrl mujoco)
 