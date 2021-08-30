option(ENABLE_CLANG_TIDY "Enable static analysis with clang-tidy" OFF)
option(ENABLE_INCLUDE_WHAT_YOU_USE "Enable static analysis with include-what-you-use" OFF)
option(ENABLE_CPP_CHECK "Enable static analysis with cppcheck" OFF)

if (ENABLE_CPP_CHECK)
  find_program(CPP_CHECK cppcheck)
  if (CPP_CHECK)
    # can't use ${GTEST_INCLUDE_DIR}, despite found gtest sources under `/usr/src/googletest` GTEST_INCLUDE_DIR is set to /usr/include
    set(CMAKE_CXX_CPPCHECK ${CPP_CHECK} --project=${CMAKE_BINARY_DIR}/compile_commands.json --enable=all --inline-suppr
                           --std=c++${CMAKE_CXX_STANDARD} --suppress=*:${CATKIN_DEVEL_PREFIX} --suppress=*:/usr/src/googletest)
  else ()
    message(SEND_ERROR "cppcheck requested but executable not found")
  endif ()
endif ()

if (ENABLE_CLANG_TIDY)
  find_program(CLANGTIDY clang-tidy)
  if (CLANGTIDY)
    set(CMAKE_CXX_CLANG_TIDY ${CLANGTIDY} --extra-arg=-Wno-unknown-warning-option -p=${CMAKE_BINARY_DIR}
                             --config-file=${CMAKE_SOURCE_DIR}/.clang-tidy)
  else ()
    message(SEND_ERROR "clang-tidy requested but executable not found")
  endif ()
endif ()

if (ENABLE_INCLUDE_WHAT_YOU_USE)
  find_program(INCLUDE_WHAT_YOU_USE include-what-you-use)
  if (INCLUDE_WHAT_YOU_USE)
    set(CMAKE_CXX_INCLUDE_WHAT_YOU_USE ${INCLUDE_WHAT_YOU_USE})
  else ()
    message(SEND_ERROR "include-what-you-use requested but executable not found")
  endif ()
endif ()
