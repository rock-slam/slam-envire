# There are some qt related things TODO: move all the vizkit related stuff into
# a cmake file in vizkit
find_package(Qt4 4.5 REQUIRED QtOpenGL)
add_definitions(${QT_DEFINITIONS})
include_directories(${QT_INCLUDE_DIR})
include_directories(${QT_QTCORE_INCLUDE_DIR})
include_directories(${QT_QTGUI_INCLUDE_DIR})
set(QT_USE_OPENGL TRUE)
include(${QT_USE_FILE})

set(QT_HEADERS 
    ItemManipulator.hpp 
    )
QT4_WRAP_CPP(moc_cpp ${QT_HEADERS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(TARGET_SRC 
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
    Uncertainty.cpp
    )

# compile shared library
add_library(${PROJECT_NAME}-viz SHARED ${TARGET_SRC} ${moc_cpp})
target_link_libraries(${PROJECT_NAME}-viz ${PROJECT_NAME}
    ${VIZKIT_LIBRARIES}
    ${Boost_THREAD_LIBRARY}
    )


add_executable(envireViewer envireViewer.cpp)
target_link_libraries(envireViewer
    ${VIZKIT_LIBRARIES}
    ${QT_QTGUI_LIBRARY}
    ${PROJECT_NAME}-viz)

# install headers and library
install(DIRECTORY . DESTINATION include/vizkit FILES_MATCHING PATTERN "*.hpp")
install(TARGETS ${PROJECT_NAME}-viz
    DESTINATION lib)

# set-up pkg file
configure_file(${PROJECT_NAME}-viz.pc.in ${PROJECT_NAME}-viz.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-viz.pc
    DESTINATION lib/pkgconfig)
