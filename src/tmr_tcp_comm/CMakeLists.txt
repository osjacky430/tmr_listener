add_library(tmr_tcp_comm tmr_tcp_comm.cpp)
target_link_libraries(tmr_tcp_comm PUBLIC catkin_lib PRIVATE project_warnings project_options)

install(TARGETS tmr_tcp_comm LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
