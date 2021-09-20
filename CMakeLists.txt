cmake_minimum_required(VERSION 3.5.1)
project(tmr_listener VERSION 0.3.1)

#####################################################################
#     Check if all plugins are built under same TMFlow version      #
#####################################################################
set(${PROJECT_NAME}_TMFLOW_SUPPORT_VERSION_MAX 1.82)

if (NOT DEFINED TMR_TMFLOW_VERSION)
  set(TMR_TMFLOW_VERSION ${${PROJECT_NAME}_TMFLOW_SUPPORT_VERSION_MAX} CACHE STRING "User-specified target TMFlow version" FORCE)
elseif (${TMR_TMFLOW_VERSION} VERSION_GREATER ${${PROJECT_NAME}_TMFLOW_SUPPORT_VERSION_MAX})
  message(FATAL_ERROR "Target version is greater than max supported" " version ${${PROJECT_NAME}_TMFLOW_SUPPORT_VERSION_MAX}.")
endif ()

# __TMR_TMFLOW_VERSION is internally used by tmr_listener
if (NOT DEFINED __TMR_TMFLOW_VERSION)
  set(__TMR_TMFLOW_VERSION ${TMR_TMFLOW_VERSION} CACHE INTERNAL "The version of TMFlow that TMR Listener is built with initially" FORCE)
else ()
  if (NOT ${__TMR_TMFLOW_VERSION} VERSION_EQUAL ${TMR_TMFLOW_VERSION})
    message(FATAL_ERROR "TMR Listener is built with TMFlow version ${__TMR_TMFLOW_VERSION} initially."
                        "To built with other version, you must call \"catkin clean tmr_listener\" and rebuild all plugins")
  endif ()
endif ()

string(REPLACE "." ";" __TMR_TMRFLOW_VERSION_COMPONENT ${__TMR_TMFLOW_VERSION})
set(${PROJECT_NAME}_TMFLOW_SUPPORT_VERSION ${__TMR_TMFLOW_VERSION})
list(GET __TMR_TMRFLOW_VERSION_COMPONENT 0 ${PROJECT_NAME}_TMFLOW_SUPPORT_VERSION_MAJOR)
list(GET __TMR_TMRFLOW_VERSION_COMPONENT 1 ${PROJECT_NAME}_TMFLOW_SUPPORT_VERSION_MINOR)
configure_file(${PROJECT_SOURCE_DIR}/include/version.hpp.in ${PROJECT_SOURCE_DIR}/include/version.hpp @ONLY)

#############################################################
#       Generate custom ethernet message if specified       #
#############################################################
if (DEFINED TM_ETHERNET_SLAVE_XML OR EXISTS ${PROJECT_SOURCE_DIR}/msg/TMREthernet.msg)
  set(TM_ETHERNET_MSG TMREthernet.msg)

  if (DEFINED TM_ETHERNET_SLAVE_XML)
    execute_process(COMMAND ${PROJECT_SOURCE_DIR}/scripts/parse_tm_ethernet_xml.py --xml-dir=${TM_ETHERNET_SLAVE_XML}
                            --msg-dir=${PROJECT_SOURCE_DIR}/msg/${TM_ETHERNET_MSG} RESULT_VARIABLE result)
  endif ()

  if (result)
    message(FATAL_ERROR "Generating TMEthernet.msg failed: ${result}")
  endif ()

  add_message_files(FILES ${TM_ETHERNET_MSG})
endif ()

# INTERPROCEDURAL_OPTIMIZATION is enforced when enabled. This policy takes effect whenever the IPO property is enabled.
# The OLD behavior for this policy is to add IPO flags only for Intel compiler on Linux. The NEW behavior for this policy
# is to add IPO flags for the current compiler or produce an error if CMake does not know the flags.
if (POLICY CMP0069)
  cmake_policy(SET CMP0069 NEW)
endif ()

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(CCache)
include(StandardProjectSetting)
include(CatkinExtension)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp message_generation pluginlib)

# some of the components are used by catkin internally, but I don't want to pull in all
# catkin dependencies to a target just for one or two components, plus it makes no sense.
# Therefore, these components are found explicitly.
find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(FILES JsonData.msg JsonDataArray.msg SubCmdDataMsg.msg)

## Generate services in the 'srv' folder
add_service_files(FILES EthernetSlaveCmd.srv)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(INCLUDE_DIRS include LIBRARIES tmr_listener CATKIN_DEPENDS roscpp message_runtime pluginlib #  DEPENDS system_lib
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# need to make sure this is done right, come back to this later
# install(DIRECTORY include/tmr_listener include/tmr_utility include/tmr_ext_script
#         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/..)

include(StaticAnalyzers)

include(Sanitizers)
add_library(project_options INTERFACE)
enable_sanitizers(project_options)

include(CompilerWarning)
add_library(project_warnings INTERFACE)
set_project_warnings(project_warnings)

add_library(catkin_lib INTERFACE)
target_link_libraries(catkin_lib INTERFACE ${catkin_LIBRARIES})
target_include_directories(catkin_lib SYSTEM INTERFACE ${catkin_INCLUDE_DIRS})
target_include_directories(catkin_lib INTERFACE ${PROJECT_SOURCE_DIR}/include)

add_subdirectory(src)

option(TMR_ENABLE_TESTING "enable tm robot unit testing" OFF)
if (TMR_ENABLE_TESTING)
  add_subdirectory(test)

  if (ENABLE_COVERAGE)
    catkin_add_gcov_report(
      REPORT_NAME "coverage.xml" GCOV_EXCLUDE ".*test.*" EXCLUDE_SRC "${PROJECT_SOURCE_DIR}/src/tmr_cmd_line_option.hpp"
      EXTRA_OPTIONS --xml-pretty --exclude-unreachable-branches --exclude-throw-branches)
  endif ()
endif ()
