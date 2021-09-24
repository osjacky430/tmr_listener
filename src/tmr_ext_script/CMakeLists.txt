add_library(tmr_ext_script detail/tmr_header_tag.cpp)
target_include_directories(tmr_ext_script PUBLIC ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(tmr_ext_script PRIVATE project_warnings project_options Boost::boost)
