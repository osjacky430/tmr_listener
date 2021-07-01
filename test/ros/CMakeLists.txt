add_library(tmr_listener_handle_mock tmr_listener_handle_mock.cpp)
target_link_libraries(tmr_listener_handle_mock PUBLIC tmr_listener gmock project_options)

add_subdirectory(test_double)

add_library(tmr_main_rostest tmr_main_rostest.cpp)
target_include_directories(tmr_main_rostest PUBLIC ${catkin_INCLUDE_DIRS})

find_package(rostest REQUIRED)

# The order of rostest is unclear to me. However, changing the order of the following two lines will change the order of
# unit testing. The desire unit test order is tmr_shutdown_rostest -> tmr_listen_comm_rostest. (@todo: stop on fail)
add_rostest_gmock(tmr_comm_rostest tmr_comm_rostest.test tmr_listen_comm_rostest.cpp tmr_eth_comm_rostest.cpp)
add_rostest(tmr_shutdown_rostest.test)
target_link_libraries(tmr_comm_rostest tmr_main_rostest tmr_listener fake_server)
