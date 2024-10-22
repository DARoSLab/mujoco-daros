include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Utils)
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_BINARY_DIR})
include_directories("/usr/local/include/eigen3")
include_directories("/usr/local/include")

set(THIS_COM "${PROJECT_SOURCE_DIR}/DARoS-Ctrl" )
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_BINARY_DIR}/Configuration.h)

add_subdirectory(PrestoeSystem) 
add_subdirectory(Utils) 