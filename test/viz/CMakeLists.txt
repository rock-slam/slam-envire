add_executable(testWidget testWidget.cpp)
target_link_libraries(testWidget 
    envire-viz
    ${VIZKIT_LIBRARIES}
    )

add_executable(testUncertainty testUncertainty.cpp)
target_link_libraries(testUncertainty 
    envire-viz
    ${VIZKIT_LIBRARIES}
    )

add_executable(testMLS testMLS.cpp)
target_link_libraries(testMLS 
    envire-viz
    ${VIZKIT_LIBRARIES}
    )
