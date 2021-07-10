function (enable_sanitizers project_name)
  if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")

    if (ENABLE_COVERAGE)
      target_compile_options(${project_name} INTERFACE --coverage -O0)
      target_link_libraries(${project_name} INTERFACE --coverage)
    endif ()

    set(SANITIZERS "")

    option(ENABLE_ASAN "Enable address sanitizer" FALSE)
    if (ENABLE_ASAN)
      target_compile_options(${project_name} INTERFACE -O1 -fno-omit-frame-pointer -fno-optimize-sibling-calls)
      list(APPEND SANITIZERS "address")
    endif ()

    option(ENABLE_LSAN "Enable leak sanitizer" FALSE)
    if (ENABLE_LSAN)
      list(APPEND SANITIZERS "leak")
    endif ()

    option(ENABLE_UBSAN "Enable undefined behavior sanitizer" FALSE)
    if (ENABLE_UBSAN)
      list(APPEND SANITIZERS "undefined")
    endif ()

    option(ENABLE_TSAN "Enable thread sanitizer" FALSE)
    if (ENABLE_TSAN)
      if ("address" IN_LIST SANITIZERS OR "leak" IN_LIST SANITIZERS)
        message(WARNING "Thread sanitizer does not work with Address and Leak sanitizer enabled")
      else ()
        target_compile_options(${project_name} INTERFACE -O1)
        list(APPEND SANITIZERS "thread")
      endif ()
    endif ()

    option(ENABLE_MSAN "Enable memory sanitizer" FALSE)
    if (ENABLE_MSAN AND CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
      if ("address" IN_LIST SANITIZERS OR "thread" IN_LIST SANITIZERS OR "leak" IN_LIST SANITIZERS)
        message(WARNING "Memory sanitizer does not work with Address, Thread and Leak sanitizer enabled")
      else ()
        list(APPEND SANITIZERS "memory")
      endif ()
    endif ()

    string(REPLACE ";" "," LIST_OF_SANITIZERS "${SANITIZERS}")
    # This required cmake version greater than 3.12
    # list(
    #   JOIN
    #   SANITIZERS
    #   ","
    #   LIST_OF_SANITIZERS)

    if (LIST_OF_SANITIZERS)
      if (NOT "${LIST_OF_SANITIZERS}" STREQUAL "")

        target_compile_options(${project_name} INTERFACE -fsanitize=${LIST_OF_SANITIZERS})
        target_link_options(${project_name} INTERFACE -fsanitize=${LIST_OF_SANITIZERS})
      endif ()
    endif ()
  endif ()

endfunction ()
