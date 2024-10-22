include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/DynSim)
include_directories(${CMAKE_SOURCE_DIR}/3rd-parties)
include_directories(SYSTEM "../../3rd-parties/qpOASES/include")

include_directories("/usr/local/include/lcm/") # lcm includes
include_directories("/usr/local/include/eigen3")
include_directories("./")
link_directories(/usr/local/lib)


add_subdirectory(DynSim)

add_executable(run_PlanarBiped main.cpp)
target_link_libraries(run_PlanarBiped lcm daros-utils PlanarBipedSim)

