file(GLOB_RECURSE sources *.cpp)
add_library(daros-fbmodel SHARED ${sources})
target_link_libraries(daros-fbmodel daros-utils daros-urdf)
