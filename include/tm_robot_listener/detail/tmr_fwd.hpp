#ifndef TMR_FWD_HPP_
#define TMR_FWD_HPP_

namespace tm_robot_listener {

template <typename T>
struct Expression;

template <typename T>
struct Variable;

template <typename T>
struct Command;

struct TMSCTResponse;
struct TMSTAResponse;
struct CPERRResponse;

}  // namespace tm_robot_listener

namespace tm_robot_listener {
namespace motion_function {

struct ScriptExit;
struct End;

template <typename Tag>
class Header;

template <typename Tag>
class HeaderProductBuilder;

}  // namespace motion_function
}  // namespace tm_robot_listener

namespace tm_robot_listener {
namespace motion_function {
namespace detail {

struct MotionFnCallPrinter;
struct SubCmdCallPrinter;

struct TMSCTTag;
struct TMSTATag;
struct CPERRTag;

}  // namespace detail
}  // namespace motion_function
}  // namespace tm_robot_listener

#endif