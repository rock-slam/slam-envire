project(ENVIRE)
set(PROJECT_VERSION 1.0)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

cmake_minimum_required(VERSION 2.6)
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

find_package(CGAL REQUIRED)
add_definitions(${CGAL_DEFINITIONS})
link_directories(${CGAL_LIBRARIES})

set(ENVMAP_HDRS
    envire/ScanMeshing.hpp
    envire/Projection.hpp
    envire/LaserScan.hpp
    envire/TriMesh.hpp
    envire/Pointcloud.hpp
    envire/Grid.hpp
    envire/Grids.hpp
    envire/Core.hpp
    envire/Image.hpp)

set(ENVMAP_SRCS
    envire/Environment.cpp
    envire/FrameNode.cpp
    envire/Layer.cpp
    envire/Operator.cpp
    envire/Serialization.cpp
    envire/SerializationFactory.cpp
    envire/ScanMeshing.cpp
    envire/TriMesh.cpp
     envire/Grid.cpp
    envire/Pointcloud.cpp
    envire/LaserScan.cpp
    envire/EventListener.cpp
    envire/Image.cpp
    envire/Grids.cpp
    )

add_library(envire SHARED ${ENVMAP_SRCS})

install(TARGETS envire
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)
install(FILES ${ENVMAP_HDRS} DESTINATION include/envire)
target_link_libraries(envire 
    ${Boost_FILESYSTEM_LIBRARY}
    ${LIBYAML_LIBRARIES}
    ${CGAL_LIBRARIES}
    ${GDAL_LIBRARIES} )

configure_file(Doxyfile.in Doxyfile @ONLY)
add_custom_target(doc doxygen Doxyfile)

configure_file(envire.pc.in envire.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/envire.pc DESTINATION lib/pkgconfig)

include_directories(${PROJECT_SOURCE_DIR})
add_subdirectory(test)

add_subdirectory(icp)
include_directories(${PROJECT_SOURCE_DIR}/icp)
link_directories(${PROJECT_BINARY_DIR}/icp)

add_executable(test_icp test_icp.cpp)
target_link_libraries(test_icp icp envire)

add_executable(env_icp tools/env_icp.cpp)
target_link_libraries(env_icp icp envire)

add_executable(env_dem tools/env_dem.cpp)
target_link_libraries(env_dem icp envire)