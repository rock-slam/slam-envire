rock_library(icp
    icp.cpp
    stability.cpp
    icpLocalization.cpp
    DEPS_CMAKE KDTree Boost)

pkg_check_modules(DFKI_TYPES REQUIRED "base-types")
include_directories(${DFKI_TYPES_INCLUDE_DIRS})

install(FILES icp.hpp
stability.hpp
icpConfigurationTypes.hpp
icpLocalization.hpp
    DESTINATION include/envire)

