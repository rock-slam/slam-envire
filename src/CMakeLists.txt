include(${PROJECT_SOURCE_DIR}/cmake/UseCGAL.cmake)
if (NOT CGAL_FOUND)
    message(FATAL_ERROR "cannot find CGAL")
endif()

if( TAUCS_FOUND )
    list(APPEND ADDITIONAL_SOURCES operators/SurfaceReconstruction.cpp)
endif( TAUCS_FOUND )

include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR})

rock_library(envire
    core/Environment.cpp
    core/FrameNode.cpp
    core/Transform.cpp
    core/Layer.cpp
    core/Operator.cpp
    core/Serialization.cpp
    core/SerializationFactory.cpp
    core/Event.cpp
    core/EventSource.cpp
    core/EventHandler.cpp
    maps/TriMesh.cpp
    maps/GridBase.cpp
    maps/Grids.cpp
    maps/Pointcloud.cpp
    maps/LaserScan.cpp
    maps/MLSGrid.cpp
    maps/MLSMap.cpp
    operators/Projection.cpp
    operators/MLSProjection.cpp
    operators/MergeMLS.cpp
    operators/MergePointcloud.cpp
    operators/ScanMeshing.cpp
    operators/SimplifyPointcloud.cpp
    operators/MLSSlope.cpp
    operators/MLSSimpleTraversability.cpp
    operators/DistanceGridToPointcloud.cpp
    tools/PlyFile.cpp
    tools/GridAccess.cpp
    tools/GraphViz.cpp
    ${ADDITIONAL_SOURCES}
    HEADERS Core.hpp
    DEPS_PKGCONFIG ply base-types base-lib
    DEPS_CMAKE LibYAML GDAL Eigen2)

install(FILES core/EventHandler.hpp
    core/Event.hpp
    core/EventSource.hpp
    DESTINATION include/envire/core)

install(FILES maps/GridBase.hpp
    maps/Grid.hpp
    maps/Grids.hpp
    maps/LaserScan.hpp
    maps/MLSGrid.hpp
    maps/MLSMap.hpp
    maps/MultiLevelSurfaceGrid.hpp
    maps/Pointcloud.hpp
    maps/TriMesh.hpp
    DESTINATION include/envire/maps)

install(FILES operators/MergeMLS.hpp
    operators/MergePointcloud.hpp
    operators/MLSProjection.hpp
    operators/MLSSimpleTraversability.hpp
    operators/MLSSlope.hpp
    operators/DistanceGridToPointcloud.hpp
    operators/Projection.hpp
    operators/ScanMeshing.hpp
    operators/SimplifyPointcloud.hpp
    operators/SurfaceReconstruction.hpp
    DESTINATION include/envire/operators)

install(FILES tools/GraphViz.hpp
    tools/GridAccess.hpp
    tools/Numeric.hpp
    tools/PlyFile.hpp
    DESTINATION include/envire/tools)

target_link_libraries(envire 
    ${CGAL_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY})

