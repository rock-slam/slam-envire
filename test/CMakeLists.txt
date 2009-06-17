find_package(Boost REQUIRED)
include_directories(${BOOST_INCLUDE_DIR})

set(SRCS unit_test.cpp)
add_executable(unit_test ${SRCS})
target_link_libraries(unit_test envire)
