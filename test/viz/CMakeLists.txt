find_package(Qt4 4.5 REQUIRED QtOpenGL)
add_definitions(${QT_DEFINITIONS})
include_directories(${QT_INCLUDE_DIR})
include_directories(${QT_QTCORE_INCLUDE_DIR})
include_directories(${QT_QTGUI_INCLUDE_DIR})
set(QT_USE_OPENGL TRUE)
include(${QT_USE_FILE})

add_executable(testWidget testWidget.cpp)
target_link_libraries(testWidget 
    envire-viz
    ${VIZKIT_LIBRARIES}
    ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY}
    )

add_executable(testUncertainty testUncertainty.cpp)
target_link_libraries(testUncertainty 
    envire-viz
    ${VIZKIT_LIBRARIES}
    ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY}
    )

add_executable(testMLS testMLS.cpp)
target_link_libraries(testMLS 
    envire-viz
    ${VIZKIT_LIBRARIES}
    ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY}
    )
