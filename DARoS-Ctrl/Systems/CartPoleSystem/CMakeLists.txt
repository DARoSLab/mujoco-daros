add_library(cart-pole_system SHARED CartPoleDynamics.cpp)
target_link_libraries(cart-pole_system lcm daros-utils)

add_executable(run_cart-pole main.cpp)
target_link_libraries(run_cart-pole cart-pole_system)

