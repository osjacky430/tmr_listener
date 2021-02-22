add_library(tm_robot_listener tm_robot_listener.cpp tmr_listener_handle.cpp)
# target_compile_options(tm_robot_listener INTERFACE -Wfatal-errors)
target_compile_definitions(tm_robot_listener PUBLIC FUSION_MAX_VECTOR_SIZE=20)
target_include_directories(tm_robot_listener PUBLIC ${catkin_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(tm_robot_listener PUBLIC ${catkin_LIBRARIES})

add_executable(tm_robot_listener_node tm_robot_listener_node.cpp)
# target_compile_options(tm_robot_listener_node INTERFACE -Wfatal-errors)
target_link_libraries(tm_robot_listener_node PUBLIC tm_robot_listener)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif ()
