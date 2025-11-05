cmake_minimum_required(VERSION 3.3)
project(envire VERSION 1.0 DESCRIPTION "Environment Representation Library")
find_package(Rock)


rock_init()
rock_feature(NOCURDIR)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

rock_find_pkgconfig(EIGEN3 eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem thread system regex program_options)
include_directories(${Boost_INCLUDE_DIRS})

# make cxx standard selectable
if (NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option( USE_CGAL "Switch on CGAL related functionality" OFF )
include(${PROJECT_SOURCE_DIR}/cmake/UseCGAL.cmake)
if (USE_CGAL)
    if (NOT CGAL_FOUND)
        message (FATAL_ERROR "CGAL, BLAS and/or LAPACK could not be found, CGAL will not be used!")
    endif()

    add_definitions( -DENVIRE_USE_CGAL -DCGAL_EIGEN3_ENABLED)    
endif()

rock_export_includedir(${PROJECT_SOURCE_DIR}/src ${PROJECT_NAME})
rock_add_source_dir(icp icp)
rock_standard_layout()
add_subdirectory(tools)

