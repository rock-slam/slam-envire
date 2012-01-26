rock_executable(env_icp env_icp.cpp
    DEPS envire
    DEPS_CMAKE GDAL)

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

rock_executable(env_mls_traversability env_mls_traversability.cpp
    DEPS envire)

rock_executable(env_mls_slopes env_mls_slopes.cpp
    DEPS envire)
