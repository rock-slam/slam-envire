rock_executable(env_icp env_icp.cpp
    DEPS envire)
rock_executable(env_mls env_mls.cpp
    DEPS envire)

rock_executable(env_create env_create.cpp
    DEPS envire)
rock_executable(env_update env_update.cpp
    DEPS envire)
rock_executable(env_delete_item env_delete_item.cpp
    DEPS envire)

rock_executable(env_create_grid env_create_grid.cpp
    DEPS envire)
rock_executable(env_add_grid env_add_grid.cpp
    DEPS envire)
rock_executable(env_grid_to_mls env_grid_to_mls.cpp
    DEPS envire)
rock_executable(env_grid_illumination env_grid_illumination.cpp
    DEPS envire)
rock_executable(env_grid_add_noise env_grid_add_noise.cpp
    DEPS envire
    DEPS_PLAIN Boost_PROGRAM_OPTIONS
    )
rock_executable(env_mls_to_grid env_mls_to_grid.cpp
    DEPS envire)
rock_executable(env_mls_to_pointcloud env_mls_to_pointcloud.cpp
    DEPS envire)
rock_executable(env_mls_trajectory env_mls_trajectory.cpp
    DEPS envire)

if( USE_CGAL )
    rock_executable(env_dem env_dem.cpp
	DEPS envire)

    rock_executable(env_simplify env_simplify.cpp
	DEPS envire)

    if( TAUCS_FOUND )
	rock_executable(env_surface env_surface.cpp)
	target_link_libraries(env_surface icp envire)
    endif( TAUCS_FOUND )
endif( USE_CGAL )

rock_executable(dem_helper dem_helper.cpp
    DEPS envire)

rock_executable(env_info env_info.cpp
    DEPS envire)

rock_executable(env_graphviz env_graphviz.cpp
    DEPS envire)

rock_executable(env_traversability env_traversability.cpp
    DEPS envire)

rock_executable(env_mls_slopes env_mls_slopes.cpp
    DEPS envire)

if (CMAKE_COMPILER_IS_GNUCXX AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9)
    add_definitions(-DHAVE_STD_REGEX)
    rock_executable(env_slam6d env_slam6d.cpp
    DEPS envire
    DEPS_PLAIN Boost_FILESYSTEM Boost_SYSTEM
    )
else()
    rock_executable(env_slam6d env_slam6d.cpp
    DEPS envire
    DEPS_PLAIN Boost_FILESYSTEM Boost_SYSTEM Boost_REGEX
    )
endif()


rock_executable(asc2ply asc2ply.cpp
    DEPS envire
    )
