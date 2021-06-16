add_library(tmr_ext_script detail/tmr_header_tag.cpp)
target_include_directories(tmr_ext_script PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_compile_options(tmr_ext_script PUBLIC "-Wfatal-errors")

set_project_warnings(tmr_ext_script)
enable_sanitizers(tmr_ext_script)
