file(GLOB_RECURSE sources "*.cpp")
add_library(tello-robots SHARED ${sources})
target_link_libraries(tello-robots daros-dynamics daros-urdf) 

