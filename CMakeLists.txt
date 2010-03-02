project(ENVMAP)
set(PROJECT_VERSION 1.0)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

cmake_minimum_required(VERSION 2.6)
include(FindPkgConfig)

find_package(Eigen2 REQUIRED)
include_directories(${EIGEN2_INCLUDE_DIR})

find_package(Boost REQUIRED COMPONENTS filesystem)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_package(LibYAML REQUIRED)
include_directories(${LibYAML_INCLUDE_DIR})
link_directories(${LibYAML_LIBRARY_DIR})

set(ENVMAP_HDRS
    envire/ScanMeshing.hpp
    envire/LaserScan.hpp
    envire/TriMesh.hpp
    envire/Core.hpp)

set(ENVMAP_SRCS
    envire/Environment.cpp
    envire/FrameNode.cpp
    envire/Layer.cpp
    envire/Operator.cpp
    envire/Serialization.cpp
    envire/SerializationFactory.cpp
    envire/ScanMeshing.cpp
    envire/TriMesh.cpp
    envire/LaserScan.cpp
    envire/EventListener.cpp
    )

add_library(envire SHARED ${ENVMAP_SRCS})

install(TARGETS envire
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)
install(FILES ${ENVMAP_HDRS} DESTINATION include/envire)
target_link_libraries(envire ${Boost_FILESYSTEM_LIBRARY})
target_link_libraries(envire ${LIBYAML_LIBRARIES})

configure_file(Doxyfile.in Doxyfile @ONLY)
add_custom_target(doc doxygen Doxyfile)

configure_file(envire.pc.in envire.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/envire.pc DESTINATION lib/pkgconfig)

include_directories(${PROJECT_SOURCE_DIR})
add_subdirectory(test)


