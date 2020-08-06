# compile shared library
rock_vizkit_plugin(envire-viz
    ElevationGridVisualization.cpp
    EnvireEventListener.cpp
    EnvireVisualization.cpp
    FrameNodeManipulator.cpp
    FrameNodeVisualization.cpp
    GridVisualizationBase.cpp
    ImageRGB24Visualization.cpp
    GridVisualization.cpp
    LaserScanVisualization.cpp
    MapSegmentVisualization.cpp
    MLSVisualization.cpp
    PointcloudVisualization.cpp
    TriMeshVisualization.cpp
    TreeViewListener.cpp
    ItemManipulator.cpp
    TraversabilityGridVisualization.cpp
    MOC ItemManipulator.hpp TreeViewListener.hpp EnvireVisualization.hpp MLSVisualization.hpp PointcloudVisualization.hpp EnvireEventListener.hpp ElevationGridVisualization.hpp MapSegmentVisualization.hpp FrameNodeVisualization.hpp GridVisualization.hpp
    DEPS envire
    DEPS_PKGCONFIG base-viz
    HEADERS EnvireVisualization.hpp EnvireEventListener.hpp TreeViewListener.hpp EnvireWidget.hpp
    EnvironmentItemVisualizer.hpp)
target_link_libraries(envire-viz ${Boost_THREAD_LIBRARY})
target_link_libraries(envire-viz pthread)

rock_executable(envireViewer envireViewer.cpp
    DEPS envire-viz
    DEPS_PKGCONFIG vizkit3d)
target_link_libraries(envireViewer
    ${QT_QTGUI_LIBRARY})
    
add_subdirectory(enview)
