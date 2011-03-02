find_package(Boost REQUIRED)
include_directories(${BOOST_INCLUDE_DIR})

add_executable(test_core unit/core.cpp)
target_link_libraries(test_core envire)

add_executable(test_icp unit/icp.cpp)
target_link_libraries(test_icp icp envire)

IF( VIZKIT_FOUND )
    add_subdirectory(viz)
ENDIF( VIZKIT_FOUND )
