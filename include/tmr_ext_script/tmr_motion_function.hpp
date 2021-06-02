#ifndef TMR_MOTION_FUNCTION_HPP_
#define TMR_MOTION_FUNCTION_HPP_

#include <boost/algorithm/string_regex.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>
#include <boost/tokenizer.hpp>
#include <functional>
#include <string>
#include <vector>

#include <iostream>

#include "tmr_ext_script/detail/tmr_function.hpp"
#include "tmr_ext_script/detail/tmr_header_tag.hpp"
#include "tmr_ext_script/detail/tmr_msg_gen.hpp"
#include "tmr_variable.hpp"

namespace tmr_listener {

enum class ErrorCode { NoError, BadArgument, BadCheckSum, BadHeader, InvalidData, NotInListenNode = 0xF1 };

struct Packet {
  std::string header;
  std::size_t length;
  std::vector<std::string> data;
  std::string checksum;
};

struct TMSTAResponse {
  int subcmd_;
  std::vector<std::string> data_{};
};

struct TMSCTResponse {
  std::string id_{""};
  bool script_result_ = false;
  std::vector<int> abnormal_line_{};
};

struct CPERRResponse {
  ErrorCode err_ = ErrorCode::NoError;
};

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

namespace motion_function {

// clang-format off
#define TMR_MOTION_FUNC(name, ret_type, ...)  constexpr tmr_listener::detail::TMSTCFuncSet<ret_type, __VA_ARGS__> name { #name } 
#define TMR_SUBCMD(name, subcmd, ...)         constexpr tmr_listener::detail::TMSTAFuncSet<__VA_ARGS__> name { #subcmd }
#define TMR_HEADER(name)                      constexpr Header<tmr_listener::detail::name##Tag> name {}
#define SIGNATURE(...)                        tmr_listener::detail::Function<__VA_ARGS__>
#define RETURN_TYPE(type)                     type
#define TMR_VOID
// clang-format on

/**
 * @brief Motion function FunctionSet instances
 */
TMR_MOTION_FUNC(QueueTag, RETURN_TYPE(bool), SIGNATURE(int), SIGNATURE(int, int));
TMR_MOTION_FUNC(WaitQueueTag, RETURN_TYPE(int), SIGNATURE(int), SIGNATURE(int, int));
TMR_MOTION_FUNC(StopAndClearBuffer, RETURN_TYPE(bool), SIGNATURE(TMR_VOID));
TMR_MOTION_FUNC(Pause, RETURN_TYPE(bool), SIGNATURE(TMR_VOID));
TMR_MOTION_FUNC(Resume, RETURN_TYPE(bool), SIGNATURE(TMR_VOID));
TMR_MOTION_FUNC(PTP, RETURN_TYPE(bool), SIGNATURE(std::string, std::array<float, 6>, int, int, int, bool),
                SIGNATURE(std::string, std::array<float, 6>, int, int, int, bool, std::array<int, 3>),
                SIGNATURE(std::string, float, float, float, float, float, float, int, int, int, bool),
                SIGNATURE(std::string, float, float, float, float, float, float, int, int, int, bool, int, int, int));
TMR_MOTION_FUNC(Line, RETURN_TYPE(bool), SIGNATURE(std::string, std::array<float, 6>, int, int, int, bool),
                SIGNATURE(std::string, float, float, float, float, float, float, int, int, int, bool));
TMR_MOTION_FUNC(Circle, RETURN_TYPE(bool),
                SIGNATURE(std::string, std::array<float, 6>, std::array<float, 6>, int, int, int, bool),
                SIGNATURE(std::string, float, float, float, float, float, float, float, float, float, float, float,
                          float, int, int, int, bool));
TMR_MOTION_FUNC(PLine, RETURN_TYPE(bool), SIGNATURE(std::string, std::array<float, 6>, int, int, int),
                SIGNATURE(std::string, float, float, float, float, float, float, int, int, int));
TMR_MOTION_FUNC(Move_PTP, RETURN_TYPE(bool), SIGNATURE(std::string, std::array<float, 6>, int, int, int, bool),
                SIGNATURE(std::string, float, float, float, float, float, float, int, int, int, bool));
TMR_MOTION_FUNC(Move_Line, RETURN_TYPE(bool), SIGNATURE(std::string, std::array<float, 6>, int, int, int, bool),
                SIGNATURE(std::string, float, float, float, float, float, float, int, int, int, bool));
TMR_MOTION_FUNC(Move_PLine, RETURN_TYPE(bool), SIGNATURE(std::string, std::array<float, 6>, int, int, int),
                SIGNATURE(std::string, float, float, float, float, float, float, int, int, int));
TMR_MOTION_FUNC(ChangeBase, RETURN_TYPE(bool), SIGNATURE(std::string), SIGNATURE(std::array<float, 6>),
                SIGNATURE(float, float, float, float, float, float));
TMR_MOTION_FUNC(ChangeTCP, RETURN_TYPE(bool), SIGNATURE(std::string), SIGNATURE(std::array<float, 6>),
                SIGNATURE(std::array<float, 6>, float), SIGNATURE(std::array<float, 6>, float, std::array<float, 9>),
                SIGNATURE(float, float, float, float, float, float),
                SIGNATURE(float, float, float, float, float, float, float),
                SIGNATURE(float, float, float, float, float, float, float, float, float, float, float, float, float,
                          float, float, float));
TMR_MOTION_FUNC(ChangeLoad, RETURN_TYPE(bool), SIGNATURE(float));

TMR_MOTION_FUNC(PVTEnter, RETURN_TYPE(bool), SIGNATURE(int), SIGNATURE(TMR_VOID));
TMR_MOTION_FUNC(PVTExit, RETURN_TYPE(bool), SIGNATURE(TMR_VOID));
TMR_MOTION_FUNC(PVTPoint, RETURN_TYPE(bool), SIGNATURE(std::array<float, 6>, std::array<float, 6>, float),
                SIGNATURE(float, float, float, float, float, float, float, float, float, float, float, float, float));
TMR_MOTION_FUNC(PVTPause, RETURN_TYPE(bool), SIGNATURE(TMR_VOID));
TMR_MOTION_FUNC(PVTResume, RETURN_TYPE(bool), SIGNATURE(TMR_VOID));

TMR_SUBCMD(InExtScriptCtlMode, 00, SIGNATURE(TMR_VOID));
TMR_SUBCMD(QueueTagDone, 01, SIGNATURE(int));

}  // namespace motion_function

/**
 * @brief Header class represents the "Header" concept in TM external scripting language
 *
 * @details tmr_listener creates two global Header instances, i.e., TMSCT, and TMSTA. Also, for all motion
 *          functions, tmr_listener creates a FunctionSet instance for each of them. By doing so, we can avoid
 *          syntax error or typo.
 *
 *          With tmr_listener, user can generate listen node command easily by fluent interface (see example (1)
 *          below). Instead of typing: "$TMSCT,XX,1,QueueTag(1,1),*XX\r\n", a typo, e.g., TMSCT to TMSTA, or
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
    using detail::TMSCTTag;
    using detail::TMSTATag;
    using std::is_same;

    constexpr bool TMSCT_HEADER_TMSCT_CMD      = is_same<Tag, TMSCTTag>::value and is_same<CommandTag, TMSCTTag>::value;
    constexpr bool TMSTA_HEADER                = is_same<Tag, TMSTATag>::value;
    constexpr bool HEADER_AND_CMD_TAG_NOT_SAME = is_same<Tag, CommandTag>::value;

    static_assert(TMSTA_HEADER or not TMSCT_HEADER_TMSCT_CMD, "ID must come before any TMSCT command");
    static_assert(HEADER_AND_CMD_TAG_NOT_SAME, "Command is not usable for this header");

    HeaderProductBuilder<Tag> ret_val{};
    ret_val.append_str(t_cmd.to_str());
    return ret_val;
  }

  /**
   * @brief operator<< for the start of the fluent interface
   *
   * @param t_id  ID in TMSTC
   * @return an instance of the builder
   */
  constexpr auto operator<<(ID const& t_id) const noexcept {
    using tmr_listener::detail::TMSCTTag;
    static_assert(std::is_same<Tag, TMSCTTag>::value, "ID is only meaningful in TMSCT command");

    HeaderProductBuilder<Tag> ret_val{};
    ret_val.append_str(t_id.id_);
    return ret_val;
  }

  /**
   * @class This class holds the information of the format of the packet, and its parsing rule
   *
   */
  struct Packet {
    using DataFormat = typename Tag::DataFormat;
    std::size_t length_;
    DataFormat data_;
    std::string checksum_;

    /**
     * @brief The parsing rule of the packet of the header
     *
     * @return ParseRule
     */
    static auto& parsing_rule() noexcept {
      using boost::spirit::qi::int_;
      using boost::spirit::qi::lit;
      using boost::spirit::qi::xdigit;
      using tmr_listener::detail::ParseRule;
      using DF = DataFormat;

      static ParseRule<Packet> rule = lit(Tag::HEADER()) >> ',' >> int_ >> ',' >> DF::parsing_rule() >> ",*" >> +xdigit;
      return rule;
    }
  };

  /**
   * @brief
   *
   * @param t_input
   * @return auto
   */
  static auto parse(std::string t_input) noexcept {
    using namespace boost::spirit::qi;
    using boost::spirit::ascii::space;

    Packet ret_val;
    bool const full_match = phrase_parse(t_input.begin(), t_input.end(), Packet::parsing_rule(), space, ret_val);

    return ret_val;
  }

  friend bool operator==(Header const& /*unused*/, std::string const& t_rhs) noexcept { return Tag::HEADER() == t_rhs; }
  friend bool operator!=(Header const& /*unused*/, std::string const& t_rhs) noexcept { return Tag::HEADER() != t_rhs; }

  friend bool operator==(std::string const& t_lhs, Header const& /*unused*/) noexcept { return Tag::HEADER() == t_lhs; }
  friend bool operator!=(std::string const& t_lhs, Header const& /*unused*/) noexcept { return Tag::HEADER() != t_lhs; }
};

/**
 * @brief TMSCT and TMSTA Header instances
 */
TMR_HEADER(TMSCT);
TMR_HEADER(TMSTA);
TMR_HEADER(CPERR);

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
 *          class SomeListenNodeEventHandler final : public ListenerHandle {
 *           protected:
 *            motion_function::BaseHeaderProductPtr generate_cmd(MessageStatus const t_prev_response) override {
 *              using namespace motion_function;
 *              if (t_prev_response == MessageStatus::Responded) {
 *                // TM robot responded to the command you sent previously
 *                // do something...
 *              }
 *
 *              // TM robot hasn't responded yet
 *              return empty_command_list();  // this line inform TMRobotListener to do other action
 *            }
 *          };
 *
 * @endcode
 */
inline auto empty_command_list() noexcept { return boost::make_shared<HeaderProduct<void>>(); }

inline auto dummy_command_list(std::string t_dummy_cmd_id) noexcept {
  return TMSCT << ID{std::move(t_dummy_cmd_id)} << End();
}

using TMSCTPacket = tmr_listener::Header<tmr_listener::detail::TMSCTTag>::Packet;
using TMSTAPacket = tmr_listener::Header<tmr_listener::detail::TMSTATag>::Packet;

}  // namespace tmr_listener

BOOST_FUSION_ADAPT_STRUCT(tmr_listener::TMSCTPacket, length_, data_, checksum_)
BOOST_FUSION_ADAPT_STRUCT(tmr_listener::TMSTAPacket, length_, data_, checksum_)

#endif