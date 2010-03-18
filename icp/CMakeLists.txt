set(ICP_HDRS
    icp.hpp
    )

set(ICP_SRCS
    icp.cpp 
    )

add_library(icp SHARED ${ICP_SRCS})
target_link_libraries(icp envire)

install(TARGETS icp
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)
install(FILES ${ICP_HDRS} DESTINATION include/envire)
