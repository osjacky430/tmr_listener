add_executable(fake_listener_node fake_listener_node.cpp)
target_link_libraries(fake_listener_node PRIVATE tmr_listener_handle_mock tmr_listener project_options)

add_library(fake_server fake_server.cpp)
target_include_directories(fake_server PUBLIC ${CMAKE_SOURCE_DIR}/test/ros)
target_link_libraries(fake_server PUBLIC tmr_ext_script project_options Boost::system)
