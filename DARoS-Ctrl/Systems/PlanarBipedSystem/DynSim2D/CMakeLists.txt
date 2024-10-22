file(GLOB_RECURSE sources "*.cpp")
add_library(PlanarBipedSim SHARED ${sources})

target_link_libraries(PlanarBipedSim lcm)

