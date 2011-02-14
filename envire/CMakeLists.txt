set(ENVMAP_SRCS
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
    maps/MultiLevelSurfaceGrid.cpp
    maps/MLSMap.cpp
    operators/Projection.cpp
    operators/MLSProjection.cpp
    operators/MergeMLS.cpp
    operators/MergePointcloud.cpp
    operators/ScanMeshing.cpp
    tools/PlyFile.cpp
    tools/GridAccess.cpp
    tools/GraphViz.cpp
    )

if( TAUCS_FOUND )
    set(ENVMAP_SRCS ${ENVMAP_SRCS}
    operators/SurfaceReconstruction.cpp
    operators/SimplifyPointcloud.cpp
    )
endif( TAUCS_FOUND )

include_directories(BEFORE ${PROJECT_SOURCE_DIR}/envire)

add_library(envire SHARED ${ENVMAP_SRCS})
target_link_libraries(envire 
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY}
    ${LIBYAML_LIBRARIES}
    ${CGAL_LIBRARIES}
    ${PLY_TYPES_LIBRARIES}
    ${GDAL_LIBRARIES} )

install(TARGETS envire
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)
install(DIRECTORY . DESTINATION include/envire FILES_MATCHING PATTERN "*.hpp")
