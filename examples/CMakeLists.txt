add_executable(insMoon insMoon.cc)
target_link_libraries(insMoon myFusion ${THIRD_PARTY_LIBS})

add_executable(landerIMU landerIMU.cc)
target_link_libraries(landerIMU myFusion ${THIRD_PARTY_LIBS})

add_executable(allanData allanData.cc)
target_link_libraries(allanData myFusion ${THIRD_PARTY_LIBS})