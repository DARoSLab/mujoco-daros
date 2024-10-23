set(CMAKE_CXX_FLAGS "-O3 -no-pie -ggdb ") # no werror here!

FILE(GLOB_RECURSE headers *.hpp)
file(GLOB_RECURSE sources *.cpp)
add_library(daros-common SHARED ${sources} ${headers})

target_link_libraries(daros-common daros-utils daros-urdf)


