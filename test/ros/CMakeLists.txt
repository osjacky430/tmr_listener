add_library(tmr_listener_handle_mock tmr_listener_handle_mock.cpp)
target_link_libraries(tmr_listener_handle_mock PUBLIC gmock tmr_listener project_options)
target_compile_definitions(tmr_listener_handle_mock PUBLIC $<$<BOOL:WIN32>:GTEST_LINKED_AS_SHARED_LIBRARY=1>)

add_subdirectory(test_double)

add_library(tmr_main_rostest tmr_main_rostest.cpp)
target_link_libraries(tmr_main_rostest PUBLIC gmock catkin_lib project_warnings)
target_include_directories(tmr_main_rostest PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(tmr_main_rostest PUBLIC $<$<BOOL:WIN32>:GTEST_LINKED_AS_SHARED_LIBRARY=1>)

find_package(rostest REQUIRED)
add_subdirectory(tmr_comm)
add_subdirectory(tmr_shutdown)
