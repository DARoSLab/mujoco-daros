find_package(lcm REQUIRED)
include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/LCM-Types/cpp)

add_library(cart-pole_system STATIC CartPoleDynamics.cpp)
target_link_libraries(cart-pole_system lcm daros-utils)

add_executable(run_cart-pole main.cpp)
target_link_libraries(run_cart-pole cart-pole_system)

