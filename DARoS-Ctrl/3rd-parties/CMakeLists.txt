add_subdirectory(qpOASES)
# add_subdirectory(qpSWIFT)
# add_subdirectory(lord_imu)
add_subdirectory(urdf)
# add_subdirectory(rbdl)

if(CMAKE_SYSTEM_NAME MATCHES Linux)
    add_subdirectory(vectornav)
endif()

