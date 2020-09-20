add_executable(insMoon insMoon.cc)
target_link_libraries(insMoon myFusion ${THIRD_PARTY_LIBS})

add_executable(landerIMU landerIMU.cc)
target_link_libraries(landerIMU myFusion ${THIRD_PARTY_LIBS})

add_executable(allanData allanData.cc)
target_link_libraries(allanData myFusion ${THIRD_PARTY_LIBS})

add_executable(simSensors simSensors.cc)
target_link_libraries(simSensors myFusion ${THIRD_PARTY_LIBS})

add_executable(spkfDemo spkfDemo.cc)
target_link_libraries(spkfDemo myFusion ${THIRD_PARTY_LIBS})

add_executable(glintDemo glintDemo.cc)
target_link_libraries(glintDemo myFusion ${THIRD_PARTY_LIBS})

add_executable(scspkfDemo scspkfDemo.cc)
target_link_libraries(scspkfDemo myFusion ${THIRD_PARTY_LIBS})


