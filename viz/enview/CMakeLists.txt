
set(QT_HEADERS 
    ApplicationWindow.hpp
    ICPHandler.hpp
    CsvDialog.hpp
    )
QT5_WRAP_CPP(SOURCES_MOC ${QT_HEADERS})

set(SOURCES_UI MainWindow.ui)
QT5_WRAP_UI(SOURCES_UI_H ${SOURCES_UI})
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/src )

set(TARGET_SRC 
    ${SOURCES_MOC}
    ${SOURCES_UI}
    ApplicationWindow.cpp
    ICPHandler.cpp
    enview.cpp
    CsvDialog.cpp
    )

add_executable(enview ${TARGET_SRC})
target_link_libraries(enview
    ${VIZKIT_LIBRARIES}
    ${PROJECT_NAME}-viz
    envire
    ${VIZKIT3D_LIBRARIES}
    icp Qt::Gui)
