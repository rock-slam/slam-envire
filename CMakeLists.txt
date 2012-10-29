cmake_minimum_required(VERSION 2.6)
include($ENV{ROCK_CMAKE_MACROS}/Rock.cmake)


rock_init(envire 1.0)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

rock_find_pkgconfig(EIGEN3 eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem thread system regex)

option( USE_CGAL "Switch on CGAL related functionality" OFF )

include(${PROJECT_SOURCE_DIR}/cmake/UseCGAL.cmake)

if (USE_CGAL AND NOT CGAL_FOUND)
    message (WARNING "CGAL, BLAS and/or LAPACK could not be found, CGAL will not be used!")
endif()

if ( USE_CGAL AND CGAL_FOUND )
    message ("-- CGAL will be used, change BOOST_ROOT to /usr")
    # To make sure envire links to the default system libraries 
    # such as CGAL does, setting BOOST_ROOT here 
	add_definitions( -DENVIRE_USE_CGAL )    
else()
    include_directories(${Boost_INCLUDE_DIRS})
    link_directories(${Boost_LIBRARY_DIRS})
endif()

rock_add_source_dir(icp icp)
rock_standard_layout()
add_subdirectory(tools)

