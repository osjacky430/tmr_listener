add_library(tmr_tcp_comm tmr_tcp_comm.cpp)
target_include_directories(tmr_tcp_comm SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_include_directories(tmr_tcp_comm PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(
  tmr_tcp_comm
  PUBLIC ${catkin_LIBRARIES}
  PRIVATE project_warnings)

enable_sanitizers(tmr_tcp_comm)
