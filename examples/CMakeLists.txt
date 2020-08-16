add_executable(simpleIMU simpleIMU.cc)
target_link_libraries(simpleIMU myFusion ${THIRD_PARTY_LIBS})

add_executable(landerIMU landerIMU.cc)
target_link_libraries(landerIMU myFusion ${THIRD_PARTY_LIBS})