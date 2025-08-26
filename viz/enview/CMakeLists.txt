
set(QT_HEADERS 
    ApplicationWindow.hpp
    ICPHandler.hpp
    CsvDialog.hpp
    )

set(SOURCES_UI MainWindow.ui)

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )

set(TARGET_SRC 
    ApplicationWindow.cpp
    ICPHandler.cpp
    enview.cpp
    CsvDialog.cpp
    )

rock_qt_executable(
    TARGETPREFIX enview
    QT4_SUFFIX ""
    SOURCES ${TARGET_SRC}
    MOC ${QT_HEADERS}
    UI ${SOURCES_UI}
    LIBS_QT ${PROJECT_NAME}-viz
    LIBS_QT5 Qt5::Gui
    LIBS envire icp
    )
