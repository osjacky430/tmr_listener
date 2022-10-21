add_library(fake_server fake_server.cpp)
target_link_libraries(fake_server PUBLIC tmr_ext_script project_options project_warnings Boost::system)
