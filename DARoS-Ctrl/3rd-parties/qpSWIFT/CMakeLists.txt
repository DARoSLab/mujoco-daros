FILE(GLOB_RECURSE headers *.hh)
FILE(GLOB_RECURSE sources *.c )
include_directories("./include")

set(CMAKE_CXX_FLAGS "-O3 -no-pie -ggdb ") # no werror here!

add_library (qpSWIFT SHARED ${headers} ${sources})
target_link_libraries(qpSWIFT ${eigen})
