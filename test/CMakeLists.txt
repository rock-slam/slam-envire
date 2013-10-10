rock_testsuite(test_core unit/core.cpp
    DEPS envire
    DEPS_CMAKE GDAL)

rock_testsuite(test_icp unit/icp.cpp
    DEPS envire icp
    DEPS_CMAKE GDAL)

rock_testsuite(test_mls unit/mls.cpp
    DEPS envire
    DEPS_CMAKE GDAL)

rock_testsuite(test_maps unit/maps.cpp
    DEPS envire )

rock_testsuite(test_uncertainty unit/uncertainty.cpp
    DEPS envire
    DEPS_CMAKE GDAL)

rock_testsuite(test_serialization unit/serialization.cpp
    DEPS envire
    DEPS_CMAKE GDAL)

if( vizkit3d_FOUND )
    add_subdirectory(viz)
endif( vizkit3d_FOUND )
