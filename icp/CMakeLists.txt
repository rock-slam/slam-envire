rock_library(icp
    icp.cpp
    stability.cpp
    icpLocalization.cpp
    DEPS_CMAKE KDTree Boost
    DEPS_PKGCONFIG base-types)

install(FILES icp.hpp
ransac.hpp
stability.hpp
icpConfigurationTypes.hpp
icpLocalization.hpp
    DESTINATION include/envire)

