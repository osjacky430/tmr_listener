#ifndef TMR_ID_HPP_
#define TMR_ID_HPP_

#include "hedley.hpp"
#include "tmr_utility/tmr_constexpr_string.hpp"
#include "tmr_utility/tmr_mt_helper.hpp"

#include <stdexcept>

// forward declaration
namespace tmr_listener {

class IDView;

#if not(defined(HEDLEY_GCC_VERSION) and HEDLEY_GCC_VERSION < HEDLEY_VERSION_ENCODE(6, 0, 0))
constexpr IDView operator""_id(char const* t_str, std::size_t t_size);
#endif

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

}  // namespace detail
}  // namespace tmr_listener

namespace tmr_listener {

template <std::size_t N>
constexpr bool satisfy_id_rule(char const (&t_str)[N]) noexcept {
  return detail::satisfy_id_rule(t_str, N);
}

inline bool satisfy_id_rule(std::string const& t_str) noexcept {
  return detail::satisfy_id_rule(t_str.data(), t_str.size() + 1);
}

/**
 * @brief class to represent ID in TMSCT and TMSVR command
 *
 * @code{.cpp}
 *
 *    // compile time check
 *    TMSCT << TMR_ID("123") << ... << End(); // (1) create ID with IILF helper macro
 *
 *    // no check at all
 *    TMSCT << ID{"123"} << ... << End();
 *    TMSCT << ID{std::to_string(some_int)} << ... << End();
 *
 * @endcode
 */
struct ID {
  std::string id_;  // const?
};

/**
 * @brief class to represent ID in TMSCT and TMSVR command, this class can only be initialized via UDL, and the UDL will
 *        do compile time check, or throw if evaluated at runtime
 *
 * @code{.cpp}
 *
 *    constexpr auto name = "123"_id;
 *    TMSCT << name << ... << End();
 *
 *    // compilte time check in gcc/clang, runtime in msvc
 *    TMSCT << "123"_id << ... << End();  // (3) Initialize ID with UDL
 *
 * @endcode
 */
class IDView {
  detail::ConstString id_;

#if not(defined(HEDLEY_GCC_VERSION) and HEDLEY_GCC_VERSION < HEDLEY_VERSION_ENCODE(6, 0, 0))
  friend constexpr IDView operator""_id(char const* /*t_str*/, std::size_t /*t_size*/);
#endif

  constexpr explicit IDView(detail::ConstString const& t_str) noexcept : id_(t_str) {}

 public:
  ID operator()() const noexcept { return ID{id_.to_std_str()}; }
};

/**
 * @brief class to represent ID in TMSCT and TMSVR command, this is used for GCC version less than 5.5 due to GCC Bug
 *        67371. It is useful only if it is initialized via GCC extension UDL. Unlike IDView, IDCharSequence will always
 *        evaluate at compile time, it never throws.
 *
 * @tparam str char sequence
 */
template <char... str>
struct IDCharSequence {
  static_assert(tmr_mt_helper::variadic_and<is_alnum(str)...>::value,
                "Bad ID, should contain only alphabets or numeric");
  ID operator()() const noexcept { return ID{std::string({str...})}; }
};

/**
 * @brief A helper macro to enforce compile time ID validity check, this is just IILF.
 *        Lambda is captured so that constexpr array can be used in this case
 *
 * @code{.cpp}
 *
 *    auto const id = TMR_ID("123");
 *
 *    constexpr char id_name[] = "123"; // note: make capturing constexpr variable work across multiple version and
 *                                      // compiler are just PITA. In MSVC, you need to have version greater than
 *                                      // v19.28 VS116.9, with flag /std:c++latest. For GCC, you must specify capture
 *                                      // for version >= 8.4, for version <= 8.1 you didn't need to specify capture.
 *                                      // (Both way, specified or not, won't work in 8.3 and 8.2 ) As for clang, you
 *                                      // must specify capture no matter what.
 *
 *                                      // According to cppreference, A lambda expression can read the value of a
 *                                      // variable without capturing it if the variable is constexpr and has no
 *                                      // mutable members. So, it should be compilers' problem...? ¯\_(ツ)_/¯
 *    auto const id = TMR_ID(id_name);
 *
 * @endcode
 */
#define TMR_ID(name)                                                                          \
  [=]() {                                                                                     \
    using namespace tmr_listener;                                                             \
    static_assert(satisfy_id_rule(name), "Bad ID, should contain only alphabets or numeric"); \
    return ID{name};                                                                          \
  }()

}  // namespace tmr_listener

namespace tmr_listener {

/**
 * @brief UDL to construct IDChecker
 *
 * @details The original reason of using UDL is to "reject" non string literal (although you can bypass this rejection
 *          by calling operator""_id directly, hence the quotation mark), it was designed to throw std::invalid_argument
 *          so that it do check for invalid input if evaluated during runtime. However, due to GCC Bug 67371, we can't
 *          do this in GCC 5.5, so I have no choice but to use gcc extension.
 *
 * @param t_str String to check
 * @param t_size number of characters in string (not null terminated unless specified, i.e. "whatever\0"_id)
 * @return IDChecker
 */
#if not(defined(HEDLEY_GCC_VERSION) and HEDLEY_GCC_VERSION < HEDLEY_VERSION_ENCODE(6, 0, 0))
constexpr IDView operator""_id(char const* const t_str, std::size_t t_size) {
  if (not detail::satisfy_id_rule(t_str, t_size + 1)) {
    throw std::invalid_argument("Bad ID, should contain only alphabets or numeric");
  }
  return IDView{detail::ConstString{t_str, t_size}};
}
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
template <typename CharT, CharT... str>
constexpr auto operator""_id() {
  return IDCharSequence<str...>{};
}
#pragma GCC diagnostic pop
#endif

}  // namespace tmr_listener

#endif