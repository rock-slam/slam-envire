project(ENVIRE)
set(PROJECT_VERSION 1.0)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
add_definitions("-Wall")

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

find_package(BLAS)
if(BLAS_FOUND)
add_definitions( ${BLAS_DEFINITIONS} )
link_directories( ${BLAS_LIBRARIES_DIR} )
link_libraries( ${BLAS_LIBRARIES} )
endif(BLAS_FOUND)

find_package(LAPACK)
if(LAPACK_FOUND)
add_definitions( ${LAPACK_DEFINITIONS} )
link_directories( ${LAPACK_LIBRARIES_DIR} )
link_libraries( ${LAPACK_LIBRARIES} )
endif(LAPACK_FOUND)

find_package(TAUCS)
if(TAUCS_FOUND)
add_definitions( ${TAUCS_DEFINITIONS} )
link_directories( ${TAUCS_LIBRARIES_DIR} )
link_libraries( ${TAUCS_LIBRARIES} )
endif(TAUCS_FOUND)

find_package(CGAL REQUIRED)
add_definitions(${CGAL_DEFINITIONS})
link_directories(${CGAL_LIBRARIES})
add_definitions(-DCGAL_DISABLE_ROUNDING_MATH_CHECK)
if ( "${BLAS_DEFINITIONS}" MATCHES ".*BLAS_USE_F2C.*" )
    add_definitions( "-DCGAL_USE_F2C" )
endif()
if ( "${LAPACK_DEFINITIONS}" MATCHES ".*LAPACK_USE_F2C.*" )
    add_definitions( "-DCGAL_USE_F2C" )
endif()

pkg_check_modules(PLY_TYPES REQUIRED "ply")
include_directories(${PLY_TYPES_INCLUDE_DIRS})
link_directories(${PLY_TYPES_LIBRARY_DIRS})

set(ENVMAP_HDRS
    envire/ScanMeshing.hpp
    envire/Projection.hpp
    envire/MLSProjection.hpp
    envire/SimplifyPointcloud.hpp
    envire/SurfaceReconstruction.hpp
    envire/MergePointcloud.hpp
    envire/LaserScan.hpp
    envire/TriMesh.hpp
    envire/Pointcloud.hpp
    envire/Grid.hpp
    envire/MultiLevelSurfaceGrid.hpp
    envire/GridAccess.hpp
    envire/Grids.hpp
    envire/Core.hpp
    )

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
    envire/GridAccess.cpp
    envire/MultiLevelSurfaceGrid.cpp
    envire/Projection.cpp
    envire/MLSProjection.cpp
    envire/MergePointcloud.cpp
    envire/Pointcloud.cpp
    envire/PlyFile.cpp
    envire/LaserScan.cpp
    envire/EventListener.cpp
    envire/Grids.cpp
    )

if( TAUCS_FOUND )
    set(ENVMAP_SRCS ${ENVMAP_SRCS}
    envire/SimplifyPointcloud.cpp
    envire/SurfaceReconstruction.cpp
    )
endif( TAUCS_FOUND )

add_library(envire SHARED ${ENVMAP_SRCS})

install(TARGETS envire
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)
install(FILES ${ENVMAP_HDRS} DESTINATION include/envire)
target_link_libraries(envire 
    ${Boost_FILESYSTEM_LIBRARY}
    ${LIBYAML_LIBRARIES}
    ${CGAL_LIBRARIES}
    ${PLY_TYPES_LIBRARIES}
    ${GDAL_LIBRARIES} )

configure_file(Doxyfile.in Doxyfile @ONLY)
add_custom_target(doc doxygen Doxyfile)

configure_file(envire.pc.in envire.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/envire.pc DESTINATION lib/pkgconfig)

add_subdirectory(icp)
include_directories(${PROJECT_SOURCE_DIR}/icp)
link_directories(${PROJECT_BINARY_DIR}/icp)

include_directories(BEFORE ${PROJECT_SOURCE_DIR})
add_subdirectory(test)
add_subdirectory(tools)
