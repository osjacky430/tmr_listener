add_library(tmr_eth_slave detail/tmr_header_tag.cpp tmr_eth_slave.cpp)
target_link_libraries(tmr_eth_slave PUBLIC tmr_tcp_comm project_warnings project_options)

add_executable(tmr_eth_slave_node tmr_eth_slave_node.cpp)
target_link_libraries(tmr_eth_slave_node PUBLIC tmr_eth_slave)

install(TARGETS tmr_eth_slave_node tmr_eth_slave RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
