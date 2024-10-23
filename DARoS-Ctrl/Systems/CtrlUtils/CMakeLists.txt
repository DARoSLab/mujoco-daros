set(CMAKE_CXX_FLAGS "-O3 -no-pie -ggdb ") # no werror here!

FILE(GLOB_RECURSE headers *.hpp)
file(GLOB_RECURSE sources *.cpp)
# add_library(daros-common SHARED ${sources} ${headers})
add_library(daros-ctrl-utils STATIC ${sources} ${headers})

target_link_libraries(daros-ctrl-utils daros-utils daros-urdf)


