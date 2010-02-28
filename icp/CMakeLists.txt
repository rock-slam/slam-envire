set(ICP_HDRS
    icp.hpp
    )

set(ICP_SRCS
    icp.cpp 
    )

add_library(icp SHARED ${ICP_SRCS})

install(TARGETS icp
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)
install(FILES ${ICP_HDRS} DESTINATION include/icp)
