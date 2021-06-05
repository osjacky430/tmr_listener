#ifndef TMR_HEADER_HPP_
#define TMR_HEADER_HPP_

#include <boost/format.hpp>
#include <boost/make_shared.hpp>

#include <numeric>
#include <string>
#include <vector>

#include "tmr_utility/tmr_parser.hpp"

namespace tmr_listener {

/**
 * @brief calculate xor checksum
 *
 * @param t_data  input string to calculate checksum
 * @return checksum string in hexadecimal form without "0x"
 *
 * @note  The input string should contain "$"
 */
inline auto calculate_checksum(std::string const& t_data) noexcept {
  return (boost::format("%02X") % std::accumulate(std::next(t_data.begin()), t_data.end(), 0, std::bit_xor<>{})).str();
}

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
  virtual std::string to_str() const noexcept   = 0;
  virtual bool has_script_exit() const noexcept = 0;

  virtual ~MessageBase() = default;
};

template <typename Tag>
class Message final : public MessageBase {
 public:
  bool scriptExit_ = false;
  bool ended_      = false;

  std::vector<std::string> content_;

  /**
   * @brief This function is getter function to check if the command list is empty or not
   *
   * @return true   Command list is empty
   * @return false  Command list is not empty
   */
  bool empty() const noexcept override { return this->content_.empty(); }

  /**
   * @brief This function converts appended commands to string to send to TM listen node server
   *
   * @return command in string form
   *
   * @note one line one command only
   * @note TMSTA doesn't support multi-command, only first will be consumed and replied
   * @note bad argument for motion function will not return ERROR for TMSCT
   * @note Error > Warning for TMSCT, even warning and error happened at the same time, TMSCT only returns ERROR line
   */
  std::string to_str() const noexcept override {
    auto const data_str = Tag::assemble(this->content_);
    auto const length   = data_str.size();
    auto const result   = (boost::format("%s,%d,%s,") % Tag::NAME() % length % data_str).str();

    return result + "*" + calculate_checksum(result) + "\r\n";
  }

  /**
   * @brief This function returns if the command contains script exit
   *
   * @return true
   * @return false
   */
  bool has_script_exit() const noexcept override { return this->scriptExit_; }
};

template <typename Impl>
struct MessageBuilder {
 private:
  friend Impl;
  Message<Impl> result_;

 public:
  /**
   * @brief operator<< for implementation of fluent interface
   *
   * @param t_cmd Command to append
   * @return *this
   *
   * @note  If ScriptExit or End is appended previously, then t_cmd will be ignored, as this->ended = true or
   *        this->scriptExit_ = true.
   *
   * @todo  Due to the note above, maybe making ScriptExit a FunctionSet makes more sense?
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
  auto operator<<(ScriptExit /*unused*/) noexcept {
    Impl::template check_acceptance<ScriptExit>();

    this->result_.content_.emplace_back("ScriptExit()");
    this->result_.scriptExit_ = true;
    return boost::make_shared<Message<Impl>>(this->result_);
  }

  /**
   * @brief operator<< for implementation of fluent interface
   *
   * @param t_end  End instance, the struct is merely used for tag dispatch
   * @return share pointer of the result
   */
  auto operator<<(End /*unused*/) noexcept {
    this->result_.ended_ = true;
    return boost::make_shared<Message<Impl>>(this->result_);
  }
};

template <typename Impl>
struct Header {
  friend bool operator==(Header const& /*unused*/, std::string const& t_rhs) noexcept { return Impl::NAME() == t_rhs; }
  friend bool operator!=(Header const& /*unused*/, std::string const& t_rhs) noexcept { return Impl::NAME() != t_rhs; }
  friend bool operator==(std::string const& t_lhs, Header const& /*unused*/) noexcept { return Impl::NAME() == t_lhs; }
  friend bool operator!=(std::string const& t_lhs, Header const& /*unused*/) noexcept { return Impl::NAME() != t_lhs; }

  struct Packet {
    using DataFrame = typename Impl::DataFormat;

    std::size_t length_ = 0;
    DataFrame data_;
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

      static ParseRule<Packet> rule = Impl::NAME() >> lit(',') >> int_ >> ',' >> DF::parsing_rule() >> ",*" >> +xdigit;
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

  template <typename T>
  auto operator<<(T const& t_input) const noexcept {
    return Impl::create_builder(t_input);
  }
};

}  // namespace prototype
};  // namespace tmr_listener

#endif