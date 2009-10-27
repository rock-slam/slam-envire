find_package(GDAL REQUIRED)
include_directories(${GDAL_INCLUDE_DIRS})
link_directories(${GDAL_LIBRARY_DIRS})

add_executable(logimport LogImport.cpp)
target_link_libraries(logimport envire ${GDAL_LIBRARIES})
