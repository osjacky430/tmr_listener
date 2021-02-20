#ifndef TMR_MOTION_FUNCTION_HPP__
#define TMR_MOTION_FUNCTION_HPP__

#include <boost/format.hpp>
#include <boost/fusion/include/at_key.hpp>
#include <boost/make_shared.hpp>
#include <functional>
#include <string>
#include <unordered_set>
#include <vector>

#include "tm_robot_listener/detail/tmr_function.hpp"
#include "tm_robot_listener/detail/tmr_msg_gen.hpp"
#include "tmr_variable.hpp"

namespace tm_robot_listener {

/**
 * @brief Typedef for TM type system, see Expression-Editor-and-Listen-Node_1.76_Rev1.00_TW.pdf
 */
using Int16  = motion_function::detail::FundamentalType<std::int16_t>;
using Int32  = motion_function::detail::FundamentalType<std::int32_t>;
using Int    = Int32;
using Bool   = motion_function::detail::FundamentalType<bool>;
using Float  = motion_function::detail::FundamentalType<float>;
using String = motion_function::detail::FundamentalType<std::string>;
using Byte   = motion_function::detail::FundamentalType<std::uint8_t>;
using Double = motion_function::detail::FundamentalType<double>;

template <typename T, std::size_t N>
using Array = motion_function::detail::FundamentalType<std::array<T, N>>;

// clang-format off
template <std::size_t N> using BoolArray = Array<bool, N>; 
template <std::size_t N> using IntArray = Array<int, N>;
template <std::size_t N> using FloatArray = Array<float, N>;
template <std::size_t N> using DoubleArray = Array<double, N>;
template <std::size_t N> using ByteArray = Array<std::uint8_t, N>;
// clang-format on

enum class ErrorCode { NoError, BadArgument, BadCheckSum, BadHeader, InvalidData, NotInListenNode = 0xF1 };

struct TMSTAResponse {
  int subcmd_;
  std::vector<std::string> data_;
};

struct TMSCTResponse {
  std::string id_;
  bool script_result_;
  std::vector<int> abnormal_line_;
};

struct CPERRResponse {
  ErrorCode err_;
};

namespace motion_function {

/**
 * @brief Utility class for tag dispatch
 */
struct ScriptExit {};

/**
 * @brief Utility class for tag dispatch
 */
struct End {};

/**
 * @brief Utility class for tag dispatch
 */
struct ID {
  std::string id_{""};
};

using BaseHeaderProductPtr = boost::shared_ptr<BaseHeaderProduct>;

/**
 * @brief Header class represents the "Header" concept in TM external scripting language
 *
 * @details tm_robot_listener creates two global Header instances, i.e., TMSCT, and TMSTA. Also, for all motion
 *          functions, tm_robot_listener creates a FunctionSet instance for each of them. By doing so, we can avoid
 *          syntax error or typo.
 *
 *          With tm_robot_listener, user can generate listen node command easily, by fluent interface (see example (1)
 *          below). Instead of typing: "$TMSCT,XX,1,QueueTag(1,1),*XX\r\n". A typo, e.g., TMSCT to TMSTA, or
 *          QueueTag(1,1) to QueuTag(1,1), or wrong length, or checksum error, you name it, may ruin one's day.
 *
 *          The generation of external script message is composed of three parts: Header, Command, and End signal. For
 *          the last part, end signal, End() and ScriptExit() is used, commands cannot be appended after End() and
 *          ScriptExit(), doing so results in compile error (no known conversion). (see example (2) below)
 *
 *          Commands without End() or ScriptExit() will also result in compile error (see example (3) below). Also,
 *          TM listen node commands are tagged, meaning that it is impossible to misuse (see example (4), (5), and (6)).
 *
 * @tparam Tag  Used to prevent users from appending wrong function to the header
 *
 * @code{.cpp}
 *
 *      TMSCT << ID{"1"} << QueueTag(1, 1) << End(); // (1) Generate "$TMSCT,15,1,QueueTag(1,1),*46"
 *      TMSCT << ID{"1"} << QueueTag(1, 1) << End() << QueueTag(1, 1);  // (2) compile error: no known conversion
 *      TMSCT << ID{"1"} << QueueTag(1, 1);  // (3) compile error
 *      TMSTA << ID{"1"} << End();  // (4) compile error: ID is only meaningful in TMSCT command
 *      TMSTA << QueueTag(1, 1) << End(); // (5) compile error: Command is not usable for this header
 *      TMSTA << QueueTagDone(1) << ScriptExit(); // (6) compile error: Script exit can only be used in TMSCT
 *
 *  @endcode
 */
template <typename Tag>
struct Header {
  using Builder   = HeaderProductBuilder<Tag>;
  using HeaderCmd = Command<Tag>;

  /**
   * @brief operator<< for the start of the fluent interface
   *
   * @param t_cmd Command to append
   * @return an instance of the builder
   *
   * @note templated here because I wanted to handle the compile error myself
   */
  template <typename CommandTag>
  constexpr auto operator<<(Command<CommandTag> const& t_cmd) const noexcept {
    detail::is_cmd_operable<Tag, CommandTag>{t_cmd};

    HeaderProductBuilder<Tag> ret_val{};
    ret_val.append_command(t_cmd);
    return ret_val;
  }

  /**
   * @brief operator<< for the start of the fluent interface
   *
   * @param t_id  ID in TMSTC
   * @return an instance of the builder
   */
  constexpr auto operator<<(ID const& t_id) const noexcept {
    static_assert(std::is_same<Tag, detail::TMSCTTag>::value, "ID is only meaningful in TMSCT command");

    HeaderProductBuilder<Tag> ret_val{};
    ret_val.append_str(t_id.id_);
    return ret_val;
  }

  friend bool operator==(Header const& /*unused*/, std::string const& t_rhs) noexcept { return Tag::HEADER() == t_rhs; }
  friend bool operator!=(Header const& /*unused*/, std::string const& t_rhs) noexcept { return Tag::HEADER() != t_rhs; }

  friend bool operator==(std::string const& t_lhs, Header const& /*unused*/) noexcept { return Tag::HEADER() == t_lhs; }
  friend bool operator!=(std::string const& t_lhs, Header const& /*unused*/) noexcept { return Tag::HEADER() != t_lhs; }
};

/**
 * @brief return empty commmand list
 *
 * @details If we don't want to response to the message immediately, or the data is not ready at that moment, one may
 *          possibily send empty command list to inform TMRobotListener.
 *
 * @return empty command list
 *
 * @code{.cpp}
 *
 *          class SomeListenNodeEventHandler : public ListenerHandle {
 *           protected:
 *            motion_function::BaseHeaderProductPtr generate_cmd(bool const t_prev_response) override {
 *              using namespace motion_function;
 *              if (t_prev_response) {  // TM robot responded to the command you send previously
 *                // do something...
 *              }
 *
 *              // TM robot hasn't responded yet
 *              return empty_command_list();  // this line inform TMRobotListener to do other action (@todo implement)
 *            }
 *          };
 *
 * @endcode
 */
inline auto empty_command_list() noexcept { return boost::make_shared<HeaderProduct<void>>(); }

// clang-format off
#define TMR_MOTION_FUNC(name, ret_type, ...)  detail::TMSTCFuncSet<ret_type, __VA_ARGS__> const name { #name } 
#define TMR_SUBCMD(name, subcmd, ...)         detail::TMSTAFuncSet<__VA_ARGS__> const name { #subcmd }
#define SIGNATURE(...)                        detail::Function<__VA_ARGS__>
#define TMR_HEADER(name)                      constexpr Header<detail::name##Tag> name {}
#define RETURN_TYPE(type)                     type
#define TMR_VOID
// clang-format on

/**
 * @brief TMSCT and TMSTA Header instances
 */
TMR_HEADER(TMSCT);
TMR_HEADER(TMSTA);
TMR_HEADER(CPERR);

/**
 * @brief Motion function FunctionSet instances
 */
TMR_MOTION_FUNC(QueueTag, RETURN_TYPE(bool), SIGNATURE(Int), SIGNATURE(Int, Int));
TMR_MOTION_FUNC(WaitQueueTag, RETURN_TYPE(int), SIGNATURE(Int), SIGNATURE(Int, Int));
TMR_MOTION_FUNC(StopAndClearBuffer, RETURN_TYPE(bool), SIGNATURE(TMR_VOID));
TMR_MOTION_FUNC(Pause, RETURN_TYPE(bool), SIGNATURE(TMR_VOID));
TMR_MOTION_FUNC(Resume, RETURN_TYPE(bool), SIGNATURE(TMR_VOID));
TMR_MOTION_FUNC(PTP, RETURN_TYPE(bool), SIGNATURE(String, FloatArray<6>, Int, Int, Int, Bool),
                SIGNATURE(String, FloatArray<6>, Int, Int, Int, Bool, IntArray<3>),
                SIGNATURE(String, Float, Float, Float, Float, Float, Float, Int, Int, Int, Bool),
                SIGNATURE(String, Float, Float, Float, Float, Float, Float, Int, Int, Int, Bool, Int, Int, Int));
TMR_MOTION_FUNC(Line, RETURN_TYPE(bool), SIGNATURE(String, FloatArray<6>, Int, Int, Int, Bool),
                SIGNATURE(String, Float, Float, Float, Float, Float, Float, Int, Int, Int, Bool));
TMR_MOTION_FUNC(Circle, RETURN_TYPE(bool), SIGNATURE(String, FloatArray<6>, FloatArray<6>, Int, Int, Int, Bool),
                SIGNATURE(String, Float, Float, Float, Float, Float, Float, Float, Float, Float, Float, Float, Float,
                          Int, Int, Int, Bool));
TMR_MOTION_FUNC(PLine, RETURN_TYPE(bool), SIGNATURE(String, FloatArray<6>, Int, Int, Int),
                SIGNATURE(String, Float, Float, Float, Float, Float, Float, Int, Int, Int));
TMR_MOTION_FUNC(Move_PTP, RETURN_TYPE(bool), SIGNATURE(String, FloatArray<6>, Int, Int, Int, Bool),
                SIGNATURE(String, Float, Float, Float, Float, Float, Float, Int, Int, Int, Bool));
TMR_MOTION_FUNC(Move_Line, RETURN_TYPE(bool), SIGNATURE(String, FloatArray<6>, Int, Int, Int, Bool),
                SIGNATURE(String, Float, Float, Float, Float, Float, Float, Int, Int, Int, Bool));
TMR_MOTION_FUNC(Move_PLine, RETURN_TYPE(bool), SIGNATURE(String, FloatArray<6>, Int, Int, Int),
                SIGNATURE(String, Float, Float, Float, Float, Float, Float, Int, Int, Int));
TMR_MOTION_FUNC(ChangeBase, RETURN_TYPE(bool), SIGNATURE(String), SIGNATURE(FloatArray<6>),
                SIGNATURE(Float, Float, Float, Float, Float, Float));
TMR_MOTION_FUNC(ChangeTCP, RETURN_TYPE(bool), SIGNATURE(String), SIGNATURE(FloatArray<6>),
                SIGNATURE(FloatArray<6>, Float), SIGNATURE(FloatArray<6>, Float, FloatArray<9>),
                SIGNATURE(Float, Float, Float, Float, Float, Float),
                SIGNATURE(Float, Float, Float, Float, Float, Float, Float),
                SIGNATURE(Float, Float, Float, Float, Float, Float, Float, Float, Float, Float, Float, Float, Float,
                          Float, Float, Float));
TMR_MOTION_FUNC(ChangeLoad, RETURN_TYPE(bool), SIGNATURE(Float));

TMR_MOTION_FUNC(PVTEnter, RETURN_TYPE(bool), SIGNATURE(Int), SIGNATURE(TMR_VOID));
TMR_MOTION_FUNC(PVTExit, RETURN_TYPE(bool), SIGNATURE(TMR_VOID));
TMR_MOTION_FUNC(PVTPoint, RETURN_TYPE(bool), SIGNATURE(FloatArray<6>, FloatArray<6>, Float),
                SIGNATURE(Float, Float, Float, Float, Float, Float, Float, Float, Float, Float, Float, Float, Float));
TMR_MOTION_FUNC(PVTPause, RETURN_TYPE(bool), SIGNATURE(TMR_VOID));
TMR_MOTION_FUNC(PVTResume, RETURN_TYPE(bool), SIGNATURE(TMR_VOID));

TMR_SUBCMD(InExtScriptCtlMode, 00, SIGNATURE(TMR_VOID));
TMR_SUBCMD(QueueTagDone, 01, SIGNATURE(Int));

}  // namespace motion_function
}  // namespace tm_robot_listener

#endif