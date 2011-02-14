add_executable(env_icp env_icp.cpp)
target_link_libraries(env_icp icp envire)

add_executable(env_dem env_dem.cpp)
target_link_libraries(env_dem envire)

add_executable(env_mls env_mls.cpp)
target_link_libraries(env_mls envire)

if( TAUCS_FOUND )
add_executable(env_simplify env_simplify.cpp)
target_link_libraries(env_simplify icp envire)

add_executable(env_surface env_surface.cpp)
target_link_libraries(env_surface icp envire)
endif( TAUCS_FOUND )

add_executable(dem_helper dem_helper.cpp)
target_link_libraries(dem_helper icp envire)

add_executable(env_info env_info.cpp)
target_link_libraries(env_info icp envire)

add_executable(env_graphviz env_graphviz.cpp)
target_link_libraries(env_graphviz icp envire)
