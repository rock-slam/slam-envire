find_package(Boost REQUIRED)
include_directories(${BOOST_INCLUDE_DIR})

set(SRCS UnitTest.cpp)
add_executable(unit_test ${SRCS})
target_link_libraries(unit_test envire)

add_executable(test_icp test_icp.cpp)
target_link_libraries(test_icp icp envire)

