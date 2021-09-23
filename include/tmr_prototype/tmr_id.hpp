#ifndef TMR_ID_HPP_
#define TMR_ID_HPP_

#include "tmr_utility/tmr_constexpr_string.hpp"

#include <stdexcept>

// forward declaration
namespace tmr_listener {

class SkipIDCheck;

namespace detail {

#if not defined(__GNUC__) and not defined(__clang__)
constexpr SkipIDCheck operator""_id(char const* const, std::size_t);
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#if defined(__clang__)
#pragma GCC diagnostic ignored "-Wgnu-string-literal-operator-template"
#endif
template <typename CharT, CharT...>
constexpr SkipIDCheck operator""_id();
#pragma GCC diagnostic pop
#endif

constexpr bool satisfy_id_rule(char const* const, std::size_t const) noexcept;

}  // namespace detail
}  // namespace tmr_listener

namespace tmr_listener {

/**
 * @brief Tag for tag dispatch, can only be constructed by UDL
 */
class SkipIDCheck {
  SkipIDCheck() = default;

 public:
#if not defined(__GNUC__) and not defined(__clang__)
  friend constexpr SkipIDCheck detail::operator""_id(char const* const, std::size_t);
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#if defined(__clang__)
#pragma GCC diagnostic ignored "-Wgnu-string-literal-operator-template"
#endif
  template <typename CharT, CharT...>
  friend constexpr SkipIDCheck detail::operator""_id();
#pragma GCC diagnostic pop
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
 * @brief class to represent ID in TMSCT and TMSVR command, since invalid ID (name with non alphabet or numeric) will
 *        result in TM robot send CPERR 04, I decided to do some validity check.
 */
struct ID {
 private:
  explicit ID(char const* const t_str, std::size_t N) : id_{t_str, N - 1} {
    if (not satisfy_id_rule(this->id_)) {
      throw std::invalid_argument("Bad ID, should contain only alphabets or numeric");
    }
  }

 public:
  std::string id_;

  template <std::size_t N>
  explicit ID(char const (&t_str)[N]) : ID(t_str, N) {
    static_assert(N > 1, "Empty string literal not allowed");
  }

  explicit ID(std::string const& t_str) : ID(t_str.data(), t_str.size() + 1) {}

  /**
   * @brief Construct ID object, with runtime check skipped, this can only be used at compile time, since SkipIDCheck
   *        can only be constructed via UDL, unless one calls the UDL directly (which is totally insane, and even if
   *        that is the case, it is equivalent as normal construct, since UDL will throw at runtime).
   *
   * @tparam N  The size of the char array
   *
   * @todo Consider assert(t_str == id_checked_struct.string_constructed_with)
   */
  template <std::size_t N>
  explicit ID(char const (&t_str)[N], SkipIDCheck /**/) noexcept : id_{t_str, N - 1} {}
};

/**
 * @brief A helper macro to enforce compile time ID validity check, this is just IILF
 *
 */
#define TMR_ID(name)                               \
  []() {                                           \
    using namespace tmr_listener::detail;          \
    constexpr auto compile_time_check = name##_id; \
    return ID{name, compile_time_check};           \
  }()

}  // namespace tmr_listener

namespace tmr_listener {
namespace detail {

constexpr bool satisfy_id_rule(char const* const t_str, std::size_t const t_size) noexcept {
  if (t_size == 1 or t_str == nullptr) {
    return false;
  }

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
 *          operator""_id directly, hence the quotation mark), it was designed to throw std::invalid_argument so that it
 *          do check for invalid input if evaluated during runtime. However, due to GCC Bug 67371, we can't do this in
 *          GCC 5.5, so I have no choice but to use gcc extension on linux.
 *
 * @param t_str String to check
 * @param t_size size of the string
 * @return SkipIDCheck
 */
#if not defined(__GNUC__) and not defined(__clang__)
constexpr SkipIDCheck operator""_id(char const* const t_str, std::size_t t_size) {
  if (not satisfy_id_rule(t_str, t_size)) {
    throw std::invalid_argument("Bad ID, should contain only alphabets or numeric");
  }
  return SkipIDCheck{};
}
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#if defined(__clang__)
#pragma GCC diagnostic ignored "-Wgnu-string-literal-operator-template"
#endif
template <typename CharT, CharT... str>
constexpr SkipIDCheck operator""_id() {
  constexpr char chr_arr[] = {str...};
  static_assert(satisfy_id_rule(chr_arr, sizeof...(str)), "Bad ID, should contain only alphabets or numeric");
  return SkipIDCheck{};
}
#pragma GCC diagnostic pop
#endif

}  // namespace detail
}  // namespace tmr_listener

#endif