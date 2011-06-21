rock_library(icp
    icp.cpp
    stability.cpp
    DEPS_CMAKE KDTree Eigen2 Boost)

install(FILES icp.hpp
stability.hpp
ICPConfigurationTypes.hpp
    DESTINATION include/envire)

