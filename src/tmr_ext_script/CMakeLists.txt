add_library(tmr_ext_script detail/tmr_header_tag.cpp)
target_include_directories(tmr_ext_script PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_compile_options(tmr_ext_script PUBLIC "-Wfatal-errors")

target_link_libraries(tmr_ext_script PRIVATE project_warnings)
enable_sanitizers(tmr_ext_script)
