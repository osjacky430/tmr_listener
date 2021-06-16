#ifndef TMR_FWD_HPP_
#define TMR_FWD_HPP_

namespace tmr_listener {

// clang-format off
template <typename Type> struct Expression;
template <typename Type> struct Variable;

template <typename Tag> class Command;
template <typename Tag> class Header;
template <typename Tag> class HeaderProductBuilder;
// clang-format on

struct TMSCTResponse;
struct TMSTAResponse;
struct CPERRResponse;
struct ScriptExit;
struct End;

enum class ErrorCode;
enum class TagNumberStatus;

}  // namespace tmr_listener

namespace tmr_listener {
namespace detail {

struct TMSCTTag;
struct TMSTATag;
struct CPERRTag;
struct MotionFnCallPrinter;
struct SubCmdCallPrinter;

template <typename Tag, typename PrintPolicy, typename RetType, typename... Functions>
struct FunctionSet;

}  // namespace detail
}  // namespace tmr_listener

#endif