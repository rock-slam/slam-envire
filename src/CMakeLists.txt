include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR})
if( USE_CGAL AND CGAL_FOUND )
    list(APPEND ADDITIONAL_SOURCES 
	operators/SimplifyPointcloud.cpp
	operators/Projection.cpp)

    if( TAUCS_FOUND )
	    list(APPEND ADDITIONAL_SOURCES operators/SurfaceReconstruction.cpp)
    endif( TAUCS_FOUND )
endif()

rock_add_public_dependencies(envire
    CMAKE GDAL)

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
    maps/ElevationGrid.cpp
    maps/Featurecloud.cpp
    maps/GridBase.cpp
    maps/Grids.cpp 
    maps/LaserScan.cpp
    maps/MLSGrid.cpp
    maps/MLSMap.cpp
    maps/MapSegment.cpp
    maps/Pointcloud.cpp
    maps/PolygonMap.cpp
    maps/TraversabilityGrid.cpp
    maps/TriMesh.cpp
    operators/Projection.cpp
    operators/MLSProjection.cpp
    operators/MergeMLS.cpp
    operators/MergePointcloud.cpp
    operators/ObjectGrowing.cpp
    operators/ScanMeshing.cpp
    operators/MLSSlope.cpp
    operators/SimpleTraversability.cpp
    operators/DistanceGridToPointcloud.cpp
    operators/Fold.cpp
    operators/GridFloatToMLS.cpp
    operators/MLSToGrid.cpp
    operators/CutPointcloud.cpp
    operators/GridIllumination.cpp 
    operators/TraversabilityGrassfire.cpp
    operators/TraversabilityGrowClasses.cpp
    operators/MLSToPointCloud.cpp
    tools/BresenhamLine.cpp
    tools/PlyFile.cpp
    tools/RadialLookUpTable.cpp
    tools/BoxLookUpTable.cpp
    tools/GridAccess.cpp
    tools/GraphViz.cpp
    ${ADDITIONAL_SOURCES}
    HEADERS Core.hpp
    DEPS_PKGCONFIG ply base-types base-lib base-logging box2d numeric
    DEPS_CMAKE LibYAML GDAL
    DEPS Boost::system Boost::filesystem Boost::thread)

install(FILES core/EventHandler.hpp
    core/Environment.hpp
    core/EnvironmentItem.hpp
    core/Event.hpp
    core/EventHandler.hpp
    core/EventSource.hpp
    core/EventTypes.hpp
    core/Features.hpp
    core/FrameNode.hpp
    core/Holder.hpp
    core/Layer.hpp
    core/Operator.hpp
    core/Serialization.hpp
    core/SerializationFactory.hpp
    core/Transform.hpp
    DESTINATION include/envire/core)

install(FILES 
    maps/ElevationGrid.hpp
    maps/Featurecloud.hpp
    maps/GridBase.hpp
    maps/Grid.hpp
    maps/Grids.hpp
    maps/LaserScan.hpp
    maps/MapSegment.hpp
    maps/MLSGrid.hpp
    maps/MLSPatch.hpp
    maps/MLSConfiguration.hpp
    maps/MLSMap.hpp
    maps/MultiLevelSurfaceGrid.hpp
    maps/Pointcloud.hpp
    maps/PolygonMap.hpp
    maps/TraversabilityGrid.hpp
    maps/TriMesh.hpp
    DESTINATION include/envire/maps)

install(FILES operators/MergeMLS.hpp
    operators/MergePointcloud.hpp
    operators/MLSProjection.hpp
    operators/ObjectGrowing.hpp
    operators/SimpleTraversability.hpp
    operators/TraversabilityGrassfire.hpp
    operators/TraversabilityGrowClasses.hpp
    operators/GridFloatToMLS.hpp
    operators/MLSToGrid.hpp
    operators/MLSSlope.hpp
    operators/DistanceGridToPointcloud.hpp
    operators/Projection.hpp
    operators/ScanMeshing.hpp
    operators/SimplifyPointcloud.hpp
    operators/SurfaceReconstruction.hpp
    operators/ClassGridProjection.hpp
    operators/CutPointcloud.hpp
    operators/GridIllumination.hpp
    operators/MLSToPointCloud.hpp
    DESTINATION include/envire/operators)

install(FILES tools/GraphViz.hpp
    tools/GridAccess.hpp
    tools/Numeric.hpp
    tools/PlyFile.hpp
    tools/GaussianMixture.hpp
    tools/ListGrid.hpp
    tools/ExpectationMaximization.hpp
    tools/BresenhamLine.hpp
    tools/VoxelTraversal.hpp
    tools/RadialLookUpTable.hpp
    DESTINATION include/envire/tools)

if (USE_CGAL AND CGAL_FOUND)
    message(WARNING "Linking boost system libraries to envire, this may lead to conflicting boost versions!")
    target_link_libraries(envire 
	    ${CGAL_LIBRARIES}
	    ${Boost_FILESYSTEM_LIBRARY}
	    ${Boost_THREAD_LIBRARY}
        ${Boost_SYSTEM_LIBRARY}
        ${Boost_THREAD_LIBRARY}
    )
else() 
    target_link_libraries(envire 
	    ${Boost_LIBRARIES})
endif()
