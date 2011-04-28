include_directories(BEFORE ${PROJECT_SOURCE_DIR}/viz)

rock_testsuite(testWidget testWidget.cpp
    DEPS envire-viz
    DEPS_PKGCONFIG vizkit)

rock_testsuite(testUncertainty testUncertainty.cpp
    DEPS envire-viz
    DEPS_PKGCONFIG vizkit)

rock_testsuite(testMLS testMLS.cpp
    DEPS envire-viz
    DEPS_PKGCONFIG vizkit)
