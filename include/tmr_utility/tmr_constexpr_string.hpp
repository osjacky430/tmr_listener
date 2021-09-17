#ifndef TMR_CONSTEXPR_STRING_HPP_
#define TMR_CONSTEXPR_STRING_HPP_

#include <string>

namespace tmr_listener {
namespace detail {

struct ConstString {
  char const* const name_;
  std::size_t const size_;

  template <std::size_t N>
  constexpr ConstString(char const (&t_name)[N]) noexcept  // NOLINT, implicit conversion is intended, for syntax sugar
    : name_{static_cast<char const* const>(t_name)}, size_{N} {
    static_assert(N > 0, "Cannot construct empty string");
  }

  explicit ConstString(std::string t_str) noexcept
    : name_{t_str.data()}, size_{t_str.back() == '\0' ? t_str.size() : t_str.size() + 1} {}

  std::string to_std_str() const noexcept { return std::string{this->name_}; }

  constexpr auto size() const noexcept { return this->size_; }

  friend bool operator==(ConstString const& t_lhs, std::string const& t_rhs) noexcept { return t_lhs.name_ == t_rhs; }
  friend bool operator==(std::string const& t_lhs, ConstString const& t_rhs) noexcept { return t_rhs.name_ == t_lhs; }
  friend bool operator!=(ConstString const& t_lhs, std::string const& t_rhs) noexcept { return t_lhs.name_ != t_rhs; }
  friend bool operator!=(std::string const& t_lhs, ConstString const& t_rhs) noexcept { return t_rhs.name_ != t_lhs; }
};

}  // namespace detail
}  // namespace tmr_listener

/**
 * @brief Some constexpr ASCII manipulation impl for compile time string check
 */
namespace tmr_listener {

constexpr bool is_digit(char t_c) noexcept { return '0' <= t_c and t_c <= '9'; }
constexpr bool is_lower(char t_c) noexcept { return 'a' <= t_c and t_c <= 'z'; }
constexpr bool is_upper(char t_c) noexcept { return 'A' <= t_c and t_c <= 'Z'; }
constexpr bool is_alpha(char t_c) noexcept { return is_lower(t_c) or is_upper(t_c); }
constexpr bool is_alnum(char t_c) noexcept { return is_alpha(t_c) or is_digit(t_c); }

}  // namespace tmr_listener

#endif