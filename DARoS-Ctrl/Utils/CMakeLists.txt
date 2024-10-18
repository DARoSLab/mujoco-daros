find_package(ZLIB REQUIRED)
include_directories(${ZLIB_INCLUDE_DIRS})

file(GLOB_RECURSE sources "*.cpp")  
add_library(daros-utils SHARED ${sources})
target_link_libraries(daros-utils lcm daros-param_handler ${ZLIB_LIBRARIES})


