#ifndef TMR_MSG_GEN_HPP__
#define TMR_MSG_GEN_HPP__

#include <boost/format.hpp>
#include <boost/fusion/include/at_key.hpp>
#include <boost/shared_ptr.hpp>
#include <numeric>
#include <string>
#include <unordered_set>

namespace tm_robot_listener {
namespace motion_function {

/**
 * @brief calculate xor checksum
 *
 * @param t_data  input string to calculate checksum
 * @return checksum string in hexadecimal form without "0x"
 *
 * @note  The input string should contain "$"
 */
inline std::string calculate_checksum(std::string const& t_data) noexcept {
  return (boost::format("%02X") % std::accumulate(std::next(t_data.begin()), t_data.end(), 0, std::bit_xor<>{})).str();
}

/**
 * @brief Base class of the command list
 *
 * @todo The final form of this class is still not yet decided, maybe it should not be a pure virtual class, cause it
 *       makes no sense to me, the purpose of doing so is to make the interface cleaner.
 */
struct BaseHeaderProduct {
  BaseHeaderProduct()                                    = default;
  BaseHeaderProduct(BaseHeaderProduct const& /*unused*/) = default;
  BaseHeaderProduct(BaseHeaderProduct&& /*unused*/)      = default;
  BaseHeaderProduct& operator=(const BaseHeaderProduct& /*unused*/) = default;
  BaseHeaderProduct& operator=(BaseHeaderProduct&&) /*unused*/ = default;

  virtual bool empty() const noexcept           = 0;
  virtual std::string to_str() const noexcept   = 0;
  virtual bool has_script_exit() const noexcept = 0;

  virtual ~BaseHeaderProduct() = default;
};

/**
 * @brief
 *
 * @tparam Tag
 */
template <typename Tag>
class HeaderProduct final : public BaseHeaderProduct {
 private:
  friend class HeaderProductBuilder<Tag>;

  bool scriptExit_ = false;
  bool ended_      = false;

  std::vector<std::string> list;

 public:
  /**
   * @brief This function is getter function to check if the command list is empty or not
   *
   * @return true   Command list is empty
   * @return false  Command list is not empty
   */
  bool empty() const noexcept override { return this->list.empty(); }

  /**
   * @brief This function converts appended commands to string to send to TM listen node server
   *
   * @return command
   * @note one line one command only
   * @note TMSTA doesn't support multi-command, only first will be consumed and replied
   * @note bad argument for motion function will not return ERROR for TMSCT
   * @note Error > Warning for TMSCT, even warning and error happened at the same time, TMSCT only returns ERROR line
   */
  std::string to_str() const noexcept override {
    auto const data_str = detail::assemble_to_msg<Tag>{}(this->list);
    auto const length   = data_str.size();
    auto const result   = (boost::format("%s,%d,%s,") % Tag::HEADER() % length % data_str).str();

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

/**
 * @brief Specialized case to represent empty response
 *
 * @tparam
 */
template <>
struct HeaderProduct<void> final : public BaseHeaderProduct {
  bool empty() const noexcept override { return true; }
  std::string to_str() const noexcept override { return ""; }
  bool has_script_exit() const noexcept override { return false; }
};

/**
 * @brief
 *
 * @tparam Tag
 */
template <typename Tag>
class HeaderProductBuilder {
 private:
  HeaderProduct<Tag> result_;
  std::unordered_set<std::string> var_list_;

  friend class Header<Tag>;
  HeaderProductBuilder() = default;

  // temp
  void append_command(Command<Tag> const& t_cmd) noexcept { this->result_.list.push_back(t_cmd.get_cmd()); }
  void append_str(std::string const& t_str) noexcept { this->result_.list.push_back(t_str); }

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
   * @note templated here because I wanted to handle the compile error myself
   * @todo maybe treat Command<Tag> only as interface, and store std::string in the background?
   */
  template <typename CommandTag>
  decltype(auto) operator<<(Command<CommandTag> const& t_cmd) noexcept {
    static_assert(std::is_same<CommandTag, motion_function::detail::TMSCTTag>::value,
                  "Only TMSCT can have multiple commands in one script");
    if (not this->result_.ended_ and not this->result_.scriptExit_) {
      this->append_command(t_cmd);
    }

    return *this;
  }

  /**
   * @brief
   *
   * @tparam T
   * @param t_var
   * @return decltype(auto)
   *
   * @todo unordered_set need to have state across header product builder
   */
  template <typename T>
  decltype(auto) operator<<(Expression<T> const& t_expr) {
    static_assert(std::is_same<Tag, motion_function::detail::TMSCTTag>::value, "Only TMSCT can declare variable");

    if (not this->result_.ended_ and not this->result_.scriptExit_) {
      this->append_str(t_expr.value);
    }

    return *this;
  }

  /**
   * @brief operator<< for implementation of fluent interface
   *
   * @param t_exit  ScriptExit instance, the struct is merely used for tag dispatch
   * @return share pointer of the result
   */
  auto operator<<(ScriptExit const& /*unused*/) noexcept {
    static_assert(std::is_same<Tag, detail::TMSCTTag>::value, "ScriptExit() can only be called in TMSCT");

    this->result_.list.emplace_back("ScriptExit()");
    this->result_.scriptExit_ = true;
    return boost::make_shared<HeaderProduct<Tag>>(this->result_);
  }

  /**
   * @brief operator<< for implementation of fluent interface
   *
   * @param t_end  End instance, the struct is merely used for tag dispatch
   * @return share pointer of the result
   */
  auto operator<<(End const& /*unused*/) noexcept {
    this->result_.ended_ = true;
    return boost::make_shared<HeaderProduct<Tag>>(this->result_);
  }
};

}  // namespace motion_function
}  // namespace tm_robot_listener

#endif