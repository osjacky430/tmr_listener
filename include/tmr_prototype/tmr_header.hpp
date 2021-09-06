#ifndef TMR_HEADER_HPP_
#define TMR_HEADER_HPP_

#include <boost/format.hpp>
#include <boost/make_shared.hpp>

#include <numeric>
#include <string>
#include <vector>

#include "tmr_utility/tmr_parser.hpp"
#include "version.hpp"

namespace tmr_listener {

/**
 * @brief calculate xor checksum
 *
 * @param t_data  input string to calculate checksum
 * @return checksum string in hexadecimal form without "0x"
 *
 * @note  The input string should contain "$"
 */
inline auto calculate_checksum(std::string const& t_data) {
  return (boost::format("%02X") % std::accumulate(std::next(t_data.begin()), t_data.end(), 0, std::bit_xor<>{})).str();
}

/**
 * @brief Utility class for tag dispatch
 */
struct ScriptExit {
#if CURRENT_TMFLOW_VERSION_GE(1, 82, 0000)
  enum class Result { ScriptFail, ScriptPass = 1 };

  struct WithPriority {
    Result result_ = Result::ScriptPass;
    explicit constexpr WithPriority(Result const t_result) noexcept : result_{t_result} {}
  };
#endif
};

#if CURRENT_TMFLOW_VERSION_GE(1, 82, 0000)

struct StopAndClearBuffer {
  enum class Option { ClearCurrentPacket = 0, ClearAndExitCurrent = 1, ClearAllPacket = 2 };
  struct WithPriority {
    Option option_ = Option::ClearCurrentPacket;
    explicit constexpr WithPriority(Option const t_option) noexcept : option_{t_option} {}
  };
};

#endif

/**
 * @brief Utility class for tag dispatch
 */
struct End {};

/**
 * @brief Utility class for tag dispatch
 */
struct ID {
  std::string id_;
};

}  // namespace tmr_listener

namespace tmr_listener {
namespace prototype {

/**
 * @brief Base class of the command list
 *
 * @todo The final form of this class is still not yet decided, maybe it should not be a pure virtual class, cause it
 *       makes no sense to me, the purpose of doing so is to make the interface cleaner.
 */
struct MessageBase {
  MessageBase()                              = default;
  MessageBase(MessageBase const& /*unused*/) = default;
  MessageBase(MessageBase&& /*unused*/)      = default;
  MessageBase& operator=(const MessageBase& /*unused*/) = default;
  MessageBase& operator=(MessageBase&&) /*unused*/ = default;

  virtual bool empty() const noexcept           = 0;
  virtual std::size_t cmd_size() const noexcept = 0;
  virtual std::string to_str() const noexcept   = 0;
  virtual bool has_script_exit() const noexcept = 0;

#if CURRENT_TMFLOW_VERSION_GE(1, 82, 0000)
  virtual bool use_priority_cmd() const noexcept = 0;
#endif

  virtual ~MessageBase() = default;
};

template <typename Tag>
struct Message final : public MessageBase {
  bool scriptExit_ = false;
  bool ended_      = false;

  std::vector<std::string> content_;

  bool empty() const noexcept override { return this->content_.empty(); }

  std::size_t cmd_size() const noexcept override { return this->content_.size(); }

  /**
   * @brief This function converts appended commands to string to send to TM listen node server
   *
   * @return command in string form
   *
   *
   * @note bad argument for motion function will not return ERROR for TMSCT
   * @note Error > Warning for TMSCT, even warning and error happened at the same time, TMSCT only returns ERROR line
   */
  std::string to_str() const noexcept override {
    auto const data_str = Tag::assemble(this->content_);
    auto const length   = data_str.size();
    auto const result   = Tag::NAME().to_std_str() + ',' + std::to_string(length) + ',' + data_str + ',';

    return result + "*" + calculate_checksum(result) + "\r\n";
  }

  bool has_script_exit() const noexcept override { return this->scriptExit_; }

#if CURRENT_TMFLOW_VERSION_GE(1, 82, 0000)
  bool is_priority_ = false;

  bool use_priority_cmd() const noexcept override { return this->is_priority_; }
#endif
};

/**
 * @brief Specialized case to represent empty response
 */
template <>
struct Message<void> final : public MessageBase {
  bool empty() const noexcept override { return true; }
  std::size_t cmd_size() const noexcept override { return 0; }
  std::string to_str() const noexcept override { return ""; }
  bool has_script_exit() const noexcept override { return false; }

#if CURRENT_TMFLOW_VERSION_GE(1, 82, 0000)
  bool use_priority_cmd() const noexcept override { return false; }
#endif
};

template <typename Impl>
struct MessageBuilder {
 private:
  friend Impl;
  Message<Impl> result_;

 public:
#if CURRENT_TMFLOW_VERSION_GE(1, 82, 0000)
  [[gnu::warn_unused_result]] auto operator<<(ScriptExit::WithPriority const t_script_exit) noexcept {
    Impl::BuildRule::check(this->result_.content_, t_script_exit);
    this->result_.scriptExit_  = true;
    this->result_.is_priority_ = true;
    return boost::make_shared<Message<Impl>>(this->result_);
  }

  [[gnu::warn_unused_result]] auto operator<<(StopAndClearBuffer::WithPriority const t_stop_clear_buffer) noexcept {
    Impl::BuildRule::check(this->result_.content_, t_stop_clear_buffer);
    this->result_.ended_       = true;
    this->result_.is_priority_ = true;
    return boost::make_shared<Message<Impl>>(this->result_);
  }
#endif

  /**
   * @brief operator<< for implementation of fluent interface
   *
   * @param t_cmd Command to append
   * @return *this
   *
   * @note  If ScriptExit or End is appended previously, then t_cmd will be ignored, as this->ended = true or
   *        this->scriptExit_ = true.
   */
  template <typename T>
  decltype(auto) operator<<(T const& t_input) noexcept {
    if (not this->result_.ended_ and not this->result_.scriptExit_) {
      Impl::BuildRule::check(this->result_.content_, t_input);  // require all type to implement to_str()
    }
    return *this;
  }

  /**
   * @brief operator<< for implementation of fluent interface
   *
   * @param t_exit  ScriptExit instance, the struct is merely used for tag dispatch
   * @return share pointer of the result
   */
  [[gnu::warn_unused_result]] auto operator<<(ScriptExit /*unused*/) noexcept {
    Impl::BuildRule::check(this->result_.content_, ScriptExit{});
    this->result_.scriptExit_ = true;
    return boost::make_shared<Message<Impl>>(this->result_);
  }

  /**
   * @brief operator<< for implementation of fluent interface
   *
   * @param t_end  End instance, the struct is merely used for tag dispatch
   * @return share pointer of the result
   */
  [[gnu::warn_unused_result]] auto operator<<(End /*unused*/) noexcept {
    this->result_.ended_ = true;
    return boost::make_shared<Message<Impl>>(this->result_);
  }
};

/**
 * @brief Header class implementation represents the "Header" concept in TM external scripting language
 *
 * @details tmr_listener creates several global Header instances, i.e., TMSCT, TMSTA and TMSVR. Also, for all motion
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
 *      TMSTA << ID{"1"} << End();  // (4) compile error: ID is only meaningful in TMSCT and TMSVR command
 *      TMSTA << QueueTag(1, 1) << End(); // (5) compile error: Command is not usable for this header
 *      TMSTA << QueueTagDone(1) << ScriptExit(); // (6) compile error: Script exit can only be used in TMSCT
 *
 *  @endcode
 */
template <typename Impl>
struct Header {
  friend bool operator==(Header const& /*unused*/, std::string const& t_rhs) noexcept { return Impl::NAME() == t_rhs; }
  friend bool operator!=(Header const& /*unused*/, std::string const& t_rhs) noexcept { return Impl::NAME() != t_rhs; }
  friend bool operator==(std::string const& t_lhs, Header const& /*unused*/) noexcept { return Impl::NAME() == t_lhs; }
  friend bool operator!=(std::string const& t_lhs, Header const& /*unused*/) noexcept { return Impl::NAME() != t_lhs; }

  /**
   * @class @class This class holds the information of the format of the packet, and its parsing rule
   */
  struct Packet {
    using DataFrame = typename Impl::DataFrame;

    std::size_t length_ = 0;
    DataFrame data_{};
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
      using DF = DataFrame;

      static ParseRule<Packet> rule = Impl::NAME().name_ >> lit(',') >> int_ >> ','  //
                                      >> DF::parsing_rule() >> ",*" >> +xdigit;
      return rule;
    }
  };

  /**
   * @brief Parse input message into Packet
   *
   * @param t_input Input string
   * @return Packet
   *
   * @todo  Should return something like "expected" for inputs that are not fully matched
   */
  static auto parse(std::string t_input) {
    using namespace boost::spirit::qi;
    using boost::spirit::ascii::space;

    Packet ret_val;
    bool const full_match = phrase_parse(t_input.begin(), t_input.end(), Packet::parsing_rule(), space, ret_val);

    return ret_val;
  }

  template <typename T>
  auto operator<<(T const& t_input) const noexcept {
    return Impl::create_builder(t_input);
  }
};

}  // namespace prototype
}  // namespace tmr_listener

#endif