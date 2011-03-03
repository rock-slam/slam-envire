project(envire)
set(PROJECT_VERSION 1.0)
cmake_minimum_required(VERSION 2.6)

# TODO move all of the cmake to the new macros
include(Rock)
include(cmake/CommonSetup.cmake)

#### Find required dependencies #################################################
include(FindPkgConfig)

find_package(KDTree REQUIRED)
include_directories(${KDTREE_INCLUDE_DIRS})

find_package(Eigen2 REQUIRED)
include_directories(${EIGEN2_INCLUDE_DIR})

find_package(Boost REQUIRED COMPONENTS filesystem thread)
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

pkg_check_modules(BASE "base-types")
include_directories(${BASE_INCLUDE_DIRS})
link_directories(${BASE_LIBRARY_DIRS})

pkg_check_modules(VIZKIT "vizkit")
include_directories(${VIZKIT_INCLUDE_DIRS})
link_directories(${VIZKIT_LIBRARY_DIRS})

include(cmake/IncludeCGAL.cmake)

#### Build Libraries #################################################
include_directories(BEFORE .)
add_subdirectory(envire)
include_directories(BEFORE envire)

add_subdirectory(icp)
include_directories(BEFORE icp)
link_directories(${PROJECT_BINARY_DIR}/icp)
rock_export_includedir(${PROJECT_SOURCE_DIR}/icp envire)

if( VIZKIT_FOUND )
    add_subdirectory(viz)
    include_directories(BEFORE viz)
endif( VIZKIT_FOUND )

add_subdirectory(test)
add_subdirectory(tools)

# Generate and install the pkg-config file
configure_file(${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}.pc.in
		${PROJECT_BINARY_DIR}/configuration/${PROJECT_NAME}.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/configuration/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig)

