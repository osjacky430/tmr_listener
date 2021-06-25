add_library(tmr_listener_handle_mock tmr_listener_handle_mock.cpp)
target_link_libraries(tmr_listener_handle_mock PUBLIC tmr_listener gmock)

add_subdirectory(test_double)

add_library(tmr_main_rostest tmr_main_rostest.cpp)
target_include_directories(tmr_main_rostest PUBLIC ${catkin_INCLUDE_DIRS})

find_package(rostest REQUIRED)
add_rostest_gmock(tmr_listen_comm_rostest tmr_rostest.test tmr_listen_comm_rostest.cpp)
target_link_libraries(
  tmr_listen_comm_rostest
  tmr_main_rostest
  tmr_listener
  fake_server)
