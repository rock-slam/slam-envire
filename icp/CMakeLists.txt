rock_library(icp
    icp.cpp
    DEPS_CMAKE KDTree Eigen2 Boost)

install(FILES icp.hpp
    DESTINATION include/envire)

