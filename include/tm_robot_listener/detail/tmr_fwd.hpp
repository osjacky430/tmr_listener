#ifndef TMR_FWD_HPP__
#define TMR_FWD_HPP__

namespace tm_robot_listener {

template <typename T>
struct Expression;

template <typename T>
struct Variable;

template <typename T>
struct Command;

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