#ifndef TMR_ID_HPP_
#define TMR_ID_HPP_

#include "tmr_utility/tmr_constexpr_string.hpp"
#include "tmr_utility/tmr_mt_helper.hpp"

#include <stdexcept>

// forward declaration
namespace tmr_listener {
namespace detail {

#if not defined(__GNUC__) and not defined(__clang__)
constexpr auto operator""_id(char const* const, std::size_t);
#endif

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

}  // namespace detail
}  // namespace tmr_listener

namespace tmr_listener {

template <std::size_t N>
constexpr bool satisfy_id_rule(char const (&t_str)[N]) noexcept {
  return detail::satisfy_id_rule(t_str, N - 1);
}

inline bool satisfy_id_rule(std::string const& t_str) noexcept {
  return detail::satisfy_id_rule(t_str.data(), t_str.size() + 1);
}

template <typename T>
struct IDChecker {
  T const& string_;
  std::size_t const size_;

  explicit constexpr IDChecker(T const& t_str) : string_{t_str}, size_{init_size(t_str)} {
    if (not detail::satisfy_id_rule(tmr_mt_helper::data(t_str), size_)) {
      throw std::invalid_argument("Bad ID");
    }
  }

 private:
  static constexpr std::size_t init_size(T const& t_str) noexcept {
    constexpr bool not_null_terminated = std::is_same<T, std::string>::value;

    auto const ret_val = tmr_mt_helper::size(t_str);
    if (not_null_terminated) {
      return ret_val + 1;  // we are checking c_string, so size of string.data() = string.size() + 1
    }
    return ret_val;
  }
};

template <>
struct IDChecker<char const* const> {
  char const* const string_;
  std::size_t const size_;

#if not defined(__GNUC__) and not defined(__clang__)
  friend constexpr auto detail::operator""_id(char const* const, std::size_t);
#endif

 private:
  /**
   * @brief The constructor of this specialization can only be called by UDL (MSVC specifically)
   */
  constexpr IDChecker(char const* const t_str, std::size_t const t_size) : string_{t_str}, size_{t_size} {}
};

/**
 * @brief class to represent ID in TMSCT and TMSVR command, since invalid ID (name with non alphabet or numeric) will
 *        result in TM robot send CPERR 04, I decided to do some validity check.
 *
 * @code{.cpp}
 *
 *    // compile time check
 *    TMSCT << TMR_ID("123") << ... << End(); // (1) create ID with IILF helper macro
 *
 *    constexpr auto id_checker = "123"_id;
 *    TMSCT << id_checker << ... << End() // (2) Initialize ID with IDChecker implicitly
 *
 *    // runtime check
 *    TMSCT << "123"_id << ... << End();  // (3) Initialize ID with IDChecker implicitly
 *    TMSCT << ID{"123"} << ... << End(); // (4) Initialize ID with IDChecker implicitly
 *    TMSCT << ID{std::to_string(some_int)} << ... << End(); //  (5) Initialize ID with IDChecker implicitly
 *
 *    TMSCT << ID{check_id("123")} << ... << End(); // (6) Initialize ID with IDChecker using check_id helper function
 *
 * @endcode
 */
struct ID {
 private:
  ID(char const* const t_string, std::size_t const t_size) noexcept : id_{t_string, t_size - 1} {}

 public:
  std::string id_;

  template <std::size_t N>
  ID(IDChecker<char[N]> const& t_valid_id)  // NOLINT
  noexcept : ID(t_valid_id.string_, N) {}

  ID(IDChecker<char const* const> const& t_valid_id) noexcept : ID(t_valid_id.string_, t_valid_id.size_) {}  //  NOLINT

  ID(IDChecker<std::string> const& t_valid_id) noexcept : id_{t_valid_id.string_} {}  // NOLINT
};

template <typename T>
constexpr auto check_id(T const& t_str) {
  return IDChecker<T>{t_str};
}

/**
 * @brief A helper macro to enforce compile time ID validity check, this is just IILF.
 *        Lambda is captured by reference so that constexpr array can be used in this case
 */
#define TMR_ID(name)                         \
  [&]() {                                    \
    using namespace tmr_listener;            \
    constexpr auto checker = check_id(name); \
    return ID{checker};                      \
  }()

}  // namespace tmr_listener

namespace tmr_listener {
namespace detail {

/**
 * @brief UDL to construct IDChecker
 *
 * @details The original reason of using UDL is to "reject" non string literal (although you can bypass this rejection
 *          by calling operator""_id directly, hence the quotation mark), it was designed to throw std::invalid_argument
 *          so that it do check for invalid input if evaluated during runtime. However, due to GCC Bug 67371, we can't
 *          do this in GCC 5.5, so I have no choice but to use gcc extension.
 *
 * @param t_str String to check
 * @param t_size size of the string
 * @return IDChecker
 */
#if not defined(__GNUC__) and not defined(__clang__)
constexpr auto operator""_id(char const* const t_str, std::size_t t_size) {
  if (not satisfy_id_rule(t_str, t_size)) {
    throw std::invalid_argument("Bad ID, should contain only alphabets or numeric");
  }
  return IDChecker<char const* const>{t_str, t_size};
}
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#if defined(__clang__)
#pragma GCC diagnostic ignored "-Wgnu-string-literal-operator-template"
#endif
template <typename CharT, CharT... str>
constexpr auto operator""_id() {
  constexpr char chr_arr[] = {str...};
  static_assert(satisfy_id_rule(chr_arr, sizeof...(str)), "Bad ID, should contain only alphabets or numeric");
  return IDChecker<decltype(chr_arr)>{chr_arr};
}
#pragma GCC diagnostic pop
#endif

}  // namespace detail
}  // namespace tmr_listener

#endif