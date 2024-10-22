set(CMAKE_CXX_FLAGS "-O3 -no-pie -ggdb ") # no werror here!

FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_library(daros-WBIC_FB SHARED ${sources} ${headers})

