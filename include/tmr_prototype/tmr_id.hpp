#ifndef TMR_ID_HPP_
#define TMR_ID_HPP_

#include "tmr_utility/tmr_constexpr_string.hpp"

#include <stdexcept>

// forward declaration
namespace tmr_listener {

class SkipIDCheck;

namespace detail {

#if _MSC_VER and _MSC_VER >= 1921
constexpr SkipIDCheck operator""_id(char const* const, std::size_t);
#else
template <typename CharT, CharT...>
constexpr SkipIDCheck operator""_id();
#endif

constexpr bool satisfy_id_rule(char const* const, std::size_t const) noexcept;

}  // namespace detail
}  // namespace tmr_listener

namespace tmr_listener {

class SkipIDCheck {
  SkipIDCheck() = default;

 public:
#if _MSC_VER and _MSC_VER >= 1921
  friend constexpr SkipIDCheck detail::operator""_id(char const* const, std::size_t);
#else
  template <typename CharT, CharT...>
  friend constexpr SkipIDCheck detail::operator""_id();
#endif
};

template <std::size_t N>
constexpr bool satisfy_id_rule(char const (&t_str)[N]) noexcept {
  return detail::satisfy_id_rule(t_str, N - 1);
}

inline bool satisfy_id_rule(std::string const& t_str) noexcept {
  return detail::satisfy_id_rule(t_str.data(), t_str.size() + 1);
}

/**
 * @brief Utility class for tag dispatch
 */
struct ID {
 private:
  explicit ID(char const* const t_str, std::size_t N) : id_{t_str, N - 1} {
    if (not satisfy_id_rule(t_str)) {
      throw std::invalid_argument("Bad ID, should contain only alphabets or numeric");
    }
  }

 public:
  std::string id_;

  template <std::size_t N>
  explicit ID(char const (&t_str)[N]) : ID(t_str, N) {}
  explicit ID(std::string const& t_str) : ID(t_str.data(), t_str.size()) {}

  template <std::size_t N>
  explicit ID(char const (&t_str)[N], SkipIDCheck /**/) noexcept : id_{t_str, N - 1} {}
};

#define TMR_ID(name)                               \
  []() {                                           \
    constexpr auto compile_time_check = name##_id; \
    return ID{name, compile_time_check};           \
  }()

}  // namespace tmr_listener

namespace tmr_listener {
namespace detail {

constexpr bool satisfy_id_rule(char const* const t_str, std::size_t const t_size) noexcept {
  for (std::size_t i = 0; i < t_size - 1; ++i) {  // NOLINT, "constexpr algorithms" is c++2a feature
    if (not is_alnum(t_str[i])) {
      return false;
    }
  }

  return true;
}

/**
 * @brief UDL to construct SkipIDCheck tag
 *
 * @details The reason of using UDL is to "reject" non string literal (although you can bypass this rejection by calling
 *          operator""_id directly, hence the quotation mark), it was designed to throw std::invalid_argument. However,
 *          due to GCC Bug 67371, we can't do this in GCC 5.5, so I have no choice but to use gcc extension on linux.
 *
 * @param t_str String to check
 * @param t_size size of the string
 * @return SkipIDCheck
 */
#if _MSC_VER and _MSC_VER >= 1921
constexpr SkipIDCheck operator""_id(char const* const t_str, std::size_t t_size) {
  if (not satisfy_id_rule(t_str, t_size)) {
    throw std::invalid_argument("Bad ID, should contain only alphabets or numeric");
  }
  return SkipIDCheck{};
}
#else
template <typename CharT, CharT... str>
constexpr SkipIDCheck operator""_id() {
  static_assert(satisfy_id_rule({str...}, sizeof...(str)), "Bad ID, should contain only alphabets or numeric");
  return SkipIDCheck{};
}
#endif

}  // namespace detail
}  // namespace tmr_listener

#endif