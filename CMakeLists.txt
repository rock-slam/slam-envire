cmake_minimum_required(VERSION 2.6)
include($ENV{ROCK_CMAKE_MACROS}/Rock.cmake)


rock_init(envire 1.0)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

rock_find_pkgconfig(EIGEN3 eigen3 REQUIRED)

option( USE_CGAL "Switch on CGAL related functionality" OFF )

if ( USE_CGAL )
    # To make sure envire links to the default system libraries 
    # such as CGAL does, setting BOOST_ROOT here 
    set(BOOST_ROOT "/usr/")
    find_package(Boost REQUIRED COMPONENTS filesystem thread system regex)

    include(${PROJECT_SOURCE_DIR}/cmake/UseCGAL.cmake)
    if( CGAL_FOUND )
	add_definitions( -DENVIRE_USE_CGAL )
    endif()
else()
    find_package(Boost REQUIRED filesystem thread system regex)
    include_directories(${Boost_INCLUDE_DIRS})
    link_directories(${Boost_LIBRARY_DIRS})
endif()

rock_add_source_dir(icp icp)
rock_standard_layout()
add_subdirectory(tools)

