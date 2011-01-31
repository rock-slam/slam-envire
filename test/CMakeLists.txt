find_package(Boost REQUIRED)
include_directories(${BOOST_INCLUDE_DIR})

set(SRCS UnitTest.cpp)
add_executable(unit_test ${SRCS})
target_link_libraries(unit_test envire)

add_executable(test_icp test_icp.cpp)
target_link_libraries(test_icp icp envire)

IF( VIZKIT_FOUND )
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
ENDIF( VIZKIT_FOUND )
