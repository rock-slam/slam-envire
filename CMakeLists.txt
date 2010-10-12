project(envire)
set(PROJECT_VERSION 1.0)
cmake_minimum_required(VERSION 2.6)

include(cmake/CommonSetup.cmake)

#### Find required dependencies #################################################
include(FindPkgConfig)

find_package(KDTree REQUIRED)
include_directories(${KDTREE_INCLUDE_DIRS})

find_package(Eigen2 REQUIRED)
include_directories(${EIGEN2_INCLUDE_DIR})

find_package(Boost REQUIRED COMPONENTS filesystem)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_package(LibYAML REQUIRED)
include_directories(${LibYAML_INCLUDE_DIR})
link_directories(${LibYAML_LIBRARY_DIR})

find_package(GDAL REQUIRED)
add_definitions(${GDAL_CFLAGS})
include_directories(${GDAL_INCLUDE_DIRS})

pkg_check_modules(PLY_TYPES REQUIRED "ply")
include_directories(${PLY_TYPES_INCLUDE_DIRS})
link_directories(${PLY_TYPES_LIBRARY_DIRS})

include(cmake/IncludeCGAL.cmake)

#### Build Libraries #################################################
include_directories(BEFORE ${PROJECT_SOURCE_DIR})
add_subdirectory(envire)

add_subdirectory(icp)
include_directories(${PROJECT_SOURCE_DIR}/icp)
link_directories(${PROJECT_BINARY_DIR}/icp)

add_subdirectory(test)
add_subdirectory(tools)
