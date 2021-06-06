add_library(tmr_eth_slave detail/tmr_header_tag.cpp tmr_eth_slave.cpp)
target_include_directories(tmr_eth_slave SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_include_directories(tmr_eth_slave PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(tmr_eth_slave PUBLIC ${catkin_LIBRARIES} tmr_tcp_comm)
target_compile_options(tmr_eth_slave PUBLIC "-Wfatal-errors")

add_executable(tmr_eth_slave_node tmr_eth_slave_node.cpp)
target_link_libraries(tmr_eth_slave_node PUBLIC tmr_tcp_comm tmr_eth_slave)
