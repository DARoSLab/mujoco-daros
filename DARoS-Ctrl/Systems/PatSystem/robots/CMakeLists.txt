file(GLOB_RECURSE sources "*.cpp")
add_library(patroclus-robots SHARED ${sources})
target_link_libraries(patroclus-robots rs-dynamics) 

