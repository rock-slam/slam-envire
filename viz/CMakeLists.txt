rock_export_includedir(${PROJECT_SOURCE_DIR}/viz vizkit)

# compile shared library
rock_vizkit_plugin(envire-viz
    ElevationGridVisualization.cpp
    EnvireEventListener.cpp
    EnvireVisualization.cpp
    FrameNodeManipulator.cpp
    FrameNodeSelection.cpp
    FrameNodeVisualization.cpp
    GridVisualizationBase.cpp
    ImageRGB24Visualization.cpp
    LaserScanVisualization.cpp
    MLSVisualization.cpp
    PointcloudVisualization.cpp
    TriMeshVisualization.cpp
    TreeViewListener.cpp
    ItemManipulator.cpp
    MOC ItemManipulator.hpp TreeViewListener.hpp 
    DEPS envire
    DEPS_CMAKE GDAL
    HEADERS EnvireVisualization.hpp EnvireEventListener.hpp TreeViewListener.hpp EnvireWidget.hpp
    EnvironmentItemVisualizer.hpp)
target_link_libraries(envire-viz ${Boost_THREAD_LIBRARY})

rock_executable(envireViewer envireViewer.cpp
    DEPS envire-viz
    DEPS_PKGCONFIG vizkit)
target_link_libraries(envireViewer
    ${QT_QTGUI_LIBRARY})
    
add_subdirectory(enview)
