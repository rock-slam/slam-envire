cmake_minimum_required(VERSION 2.6)
include($ENV{ROCK_CMAKE_MACROS}/Rock.cmake)
rock_init(envire 1.0)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Boost REQUIRED COMPONENTS filesystem thread)

rock_add_source_dir(icp icp)
rock_standard_layout()
add_subdirectory(tools)

