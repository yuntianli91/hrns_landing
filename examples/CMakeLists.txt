add_executable(simpleIMU simpleIMU.cc)
target_link_libraries(simpleIMU myFusion ${THIRD_PARTY_LIBS})

add_executable(cvIMU cvIMU.cc)
target_link_libraries(cvIMU myFusion ${THIRD_PARTY_LIBS})