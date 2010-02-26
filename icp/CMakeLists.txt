set(ICP_HDRS
    icp.hpp
    )

set(ICP_SRCS
    icp.cpp 
    )

add_library(icp SHARED ${ICP_SRCS})

add_executable(test_icp test_icp.cpp)
target_link_libraries(test_icp icp envire)

install(TARGETS icp
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)
install(FILES ${ICP_HDRS} DESTINATION include/icp)

