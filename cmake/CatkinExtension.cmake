function (catkin_add_gcov_report)
  set(options)
  set(filterArgs ROOT_DIR FILTER GCOV_FILTER GCOV_EXCLUDE EXCLUDE_SRC EXCLUDE_DIR)
  set(targetArgs TARGET WORKING_DIR)
  set(multiValueArgs OUTPUT_DIR EXTRA_OPTIONS)

  cmake_parse_arguments("" "${options}" "${targetsArgs}" "${filterArgs};${multiValueArgs}" ${ARGN})

  find_program(GCOVR gcovr)

  set(GCOVR_OUTPUT_EXTENSIONS ".xml" ".json" ".txt" ".csv" ".html")
  set(output_list)
  foreach (output_file IN LISTS _OUTPUT_DIR)
    get_filename_component(extension ${output_file} EXT)
    if (NOT ${extension} IN_LIST GCOVR_OUTPUT_EXTENSIONS)
      message(FATAL_ERROR "${extension} is not supported by gcovr")
    endif ()

    list(APPEND output_list "-o" "${output_file}")
  endforeach ()

  set(gcov_filter_list)
  foreach (gcov_filter IN LISTS _GCOV_EXCLUDE)
    list(APPEND gcov_filter_list "--gcov-exclude=\"${gcov_filter}\"")
  endforeach ()

  set(exclude_src_list)
  foreach (exclude_src IN LISTS _EXCLUDE_SRC)
    list(APPEND exclude_src_list "-e" "${exclude_src}")
  endforeach ()

  if (${_TARGET})
    add_dependencies(run_tests ${_TARGET})
  endif ()

  if (NOT ${_WORKING_DIR})
    set(working_dir ${CMAKE_BINARY_DIR})
  endif ()

  add_custom_command(
    TARGET run_tests #
    POST_BUILD COMMAND ${GCOVR} . -r ${PROJECT_SOURCE_DIR} ${_EXTRA_OPTIONS} ${output_list} ${gcov_filter_list}
                       ${exclude_src_list} #
    WORKING_DIRECTORY ${working_dir} #
    BYPRODUCTS ${_OUTPUT_DIR})

endfunction ()
