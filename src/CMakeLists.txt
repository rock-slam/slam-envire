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
    maps/TriMesh.cpp
    maps/GridBase.cpp
    maps/Grids.cpp
    maps/Pointcloud.cpp
    maps/Featurecloud.cpp
    maps/LaserScan.cpp
    maps/MLSGrid.cpp
    maps/MLSMap.cpp
    maps/MapSegment.cpp
    operators/MLSProjection.cpp
    operators/MergeMLS.cpp
    operators/MergePointcloud.cpp
    operators/ScanMeshing.cpp
    operators/MLSSlope.cpp
    operators/SimpleTraversability.cpp
    operators/DistanceGridToPointcloud.cpp
    operators/GridFloatToMLS.cpp
    operators/MLSToGrid.cpp
    operators/CutPointcloud.cpp
    operators/GridIllumination.cpp
    tools/PlyFile.cpp
    tools/GridAccess.cpp
    tools/GraphViz.cpp
    ${ADDITIONAL_SOURCES}
    HEADERS Core.hpp
    DEPS_PKGCONFIG ply base-types base-lib
    DEPS_CMAKE LibYAML GDAL
    DEPS_PLAIN Boost_FILESYSTEM Boost_THREAD)

install(FILES core/EventHandler.hpp
    core/Features.hpp
    core/Event.hpp
    core/EventSource.hpp
    core/Serialization.hpp
    core/EventTypes.hpp
    core/Transform.hpp
    core/Holder.hpp
    DESTINATION include/envire/core)

install(FILES maps/GridBase.hpp
    maps/Grid.hpp
    maps/Grids.hpp
    maps/LaserScan.hpp
    maps/MLSGrid.hpp
    maps/MLSPatch.hpp
    maps/MLSMap.hpp
    maps/MultiLevelSurfaceGrid.hpp
    maps/Pointcloud.hpp
    maps/Featurecloud.hpp
    maps/TriMesh.hpp
    maps/MapSegment.hpp
    DESTINATION include/envire/maps)

install(FILES operators/MergeMLS.hpp
    operators/MergePointcloud.hpp
    operators/MLSProjection.hpp
    operators/SimpleTraversability.hpp
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
    DESTINATION include/envire/operators)

install(FILES tools/GraphViz.hpp
    tools/GridAccess.hpp
    tools/Numeric.hpp
    tools/PlyFile.hpp
    tools/GaussianMixture.hpp
    tools/ExpectationMaximization.hpp
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
