find_package(Boost REQUIRED)
include_directories(${BOOST_INCLUDE_DIR})

set(SRCS UnitTest.cpp)
add_executable(unit_test ${SRCS})
target_link_libraries(unit_test envire)

add_executable(main_test testmain.cpp)
target_link_libraries(main_test envire)