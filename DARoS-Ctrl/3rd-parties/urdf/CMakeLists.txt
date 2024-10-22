FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)

add_library(daros-urdf SHARED ${sources} ${headers})
