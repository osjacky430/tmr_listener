add_library(tmr_eth_slave detail/tmr_header_tag.cpp tmr_eth_slave.cpp)
target_link_libraries(tmr_eth_slave PUBLIC tmr_tcp_comm PRIVATE project_warnings project_options)
target_compile_options(tmr_eth_slave PUBLIC "-Wfatal-errors")

add_executable(tmr_eth_slave_node tmr_eth_slave_node.cpp)
target_link_libraries(tmr_eth_slave_node PUBLIC tmr_tcp_comm tmr_eth_slave)
