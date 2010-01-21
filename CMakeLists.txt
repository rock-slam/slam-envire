project(ICP)
set(PROJECT_VERSION 1.0)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

cmake_minimum_required(VERSION 2.6)
include(FindPkgConfig)

find_package(KDTree REQUIRED)
include_directories(${KDTREE_INCLUDE_DIRS})

find_package(Eigen2 REQUIRED)
include_directories(${EIGEN2_INCLUDE_DIR})

add_subdirectory(icp)
include_directories(${ICP_SOURCE_DIR}/icp)
link_directories(${ICP_BINARY_DIR}/icp)

configure_file(icp.pc.in icp.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/icp.pc DESTINATION lib/pkgconfig)

