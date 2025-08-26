# compile shared library
rock_find_qt4(OPTIONAL)
rock_find_qt5(OPTIONAL)

include(RockQt)

rock_qt_vizkit_plugin(
    TARGETPREFIX envire-viz
    QT4_SUFFIX ""
    SOURCES
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
    DEPS_PKGCONFIG_QT base-viz
    HEADERS EnvireVisualization.hpp EnvireEventListener.hpp TreeViewListener.hpp EnvireWidget.hpp
    EnvironmentItemVisualizer.hpp
    LIBS ${Boost_THREAD_LIBRARY} pthread)

rock_qt_executable(
    TARGETPREFIX envireViewer
    QT4_SUFFIX ""
    SOURCES envireViewer.cpp
    DEPS_QT envire-viz
    DEPS_PKGCONFIG_QT vizkit3d
    LIBS_QT4 Qt4::QtGui
    LIBS_QT5 Qt5::Core Qt5::Gui)


add_subdirectory(enview)
