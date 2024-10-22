file(GLOB_RECURSE sources "*.cpp")
add_library(tello-observers SHARED ${sources})
target_link_libraries(tello-observers daros-utils tello-robots) 
