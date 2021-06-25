add_library(tmr_listener tmr_listener.cpp tmr_listener_handle.cpp)
target_compile_definitions(tmr_listener PUBLIC FUSION_MAX_VECTOR_SIZE=20)
target_include_directories(tmr_listener SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_include_directories(tmr_listener PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(
  tmr_listener
  PUBLIC ${catkin_LIBRARIES} tmr_tcp_comm tmr_ext_script
  PRIVATE project_warnings)
target_compile_options(tmr_listener PUBLIC "-Wfatal-errors")

add_executable(tmr_listener_node tmr_listener_node.cpp)
target_link_libraries(tmr_listener_node PUBLIC tmr_listener)

enable_sanitizers(tmr_listener)
