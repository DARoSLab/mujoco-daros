# unit test

# ------ URDF TEST
set(urdf_test_src urdf_test.cpp)
add_executable(test_urdf ${urdf_test_src})
target_link_libraries(test_urdf daros-utils daros-urdf)

# ------ RBDL TEST
set(rbdl_test_src rbdl_test.cpp)
add_executable(test_rbdl ${rbdl_test_src})
target_link_libraries(test_rbdl daros-utils daros-urdf daros-rbdl)

# ------ Prestoe model TEST
set(prestoe_model_test_src prestoe_model_test.cpp)
add_executable(test_prestoe ${prestoe_model_test_src})
target_link_libraries(test_prestoe daros-utils daros-rbdl daros-dynamics)


