project(ENVMAP)
set(PROJECT_VERSION 1.0)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

cmake_minimum_required(VERSION 2.6)
include(FindPkgConfig)

find_package(Eigen2 REQUIRED)
include_directories(${EIGEN2_INCLUDE_DIR})

set(ENVMAP_HDRS
    envire/core.h)

set(ENVMAP_SRCS
    envire/core.cpp)

add_library(envire SHARED ${ENVMAP_SRCS})

install(TARGETS envire
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)
install(FILES ${ENVMAP_HDRS} DESTINATION include)

configure_file(Doxyfile.in Doxyfile @ONLY)
add_custom_target(doc doxygen Doxyfile)

configure_file(envire.pc.in envire.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/envire.pc DESTINATION lib/pkgconfig)

include_directories(${PROJECT_SOURCE_DIR})
add_subdirectory(test)

