FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_library(daros-WBC SHARED ${sources} ${headers})
target_link_libraries(daros-WBC Goldfarb_Optimizer)

