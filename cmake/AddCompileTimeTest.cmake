function (_create_test)
  set(options SHOULD_PASS)
  set(oneValueArgs TARGET_NAME TEST_NAME EXPRESSION)
  set(multiValueArgs ERR_MSG_REGEX)

  cmake_parse_arguments("" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  add_executable(${_TARGET_NAME} tmr_msg_gen_syntax_test.cpp)
  set_target_properties(${_TARGET_NAME} PROPERTIES EXCLUDE_FROM_ALL TRUE EXCLUDE_FROM_DEFAULT_BUILD TRUE)
  target_include_directories(${_TARGET_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include)
  target_compile_definitions(${_TARGET_NAME} PRIVATE ${_TEST_NAME} TEST_EXPRESSION=${_EXPRESSION})

  include(CTest)
  add_test(NAME ${_TEST_NAME} COMMAND ${CMAKE_COMMAND} --build . --target ${_TARGET_NAME} --config $<CONFIGURATION>
           WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

  if (NOT ${_SHOULD_PASS})
    set_tests_properties(${_TEST_NAME} PROPERTIES PASS_REGULAR_EXPRESSION "${_ERR_MSG_REGEX}")
  endif ()
endfunction ()

function (test_ext_script_syntax)

  set(options SHOULD_PASS)
  set(oneValueArgs TEST_CASE PASS_EXPR FAIL_EXPR ERR_MSG_REGEX)
  set(multiValueArgs)

  cmake_parse_arguments("" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  string(TOUPPER "${_TEST_CASE}" test_name)
  string(TOLOWER "${_TEST_CASE}" target_name)

  set(pass_case "PASS_${test_name}")
  set(fail_case "FAIL_${test_name}")
  set(pass_target "pass_${target_name}")
  set(fail_target "fail_${target_name}")

  if (NOT _ERR_MSG_REGEX)
    set(err_regex "return expr;static_assert")
  else ()
    set(err_regex ${_ERR_MSG_REGEX})
  endif ()

  # cmake-format: off
  _create_test(TARGET_NAME ${pass_target} TEST_NAME ${pass_case} EXPRESSION ${_PASS_EXPR} SHOULD_PASS)
  _create_test(TARGET_NAME ${fail_target} TEST_NAME ${fail_case} EXPRESSION ${_FAIL_EXPR} ERR_MSG_REGEX ${err_regex})
  # cmake-format: on
endfunction ()
