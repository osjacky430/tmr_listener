#ifndef TMR_MT_HELPER_HPP_
#define TMR_MT_HELPER_HPP_

#include <array>
#include <boost/mpl/bool.hpp>
#include <type_traits>

namespace tmr_mt_helper {

/**
 * @brief Helper functor to determine if one type is specialization of the template type
 *
 * @tparam Test To test
 * @tparam Ref  Template type
 *
 * @note  The following way is far better than the current one. However, it fail to compile for GCC version prior
 *        than 5.3, which can't be used since it is the default version for Ubuntu 16.04,.
 *
 *        template <template <typename...> class T, typename U>
 *        static inline constexpr auto is_specialization_of = false;
 *
 *        template <template <typename...> class T, typename... Ts>
 *        static inline constexpr auto is_specialization_of<T, T<Ts...>> = true;
 */
template <typename Test, template <typename...> class Ref>
struct is_specialization_of : std::false_type {};

template <template <typename...> class Ref, typename... Args>
struct is_specialization_of<Ref<Args...>, Ref> : std::true_type {};

/**
 * @brief
 *
 * @tparam T
 */
template <typename T>
struct is_std_array : public std::false_type {};

template <typename T, std::size_t N>
struct is_std_array<std::array<T, N>> : std::true_type {};

/**
 * @brief Helper functor to do operator && with template parameter pack by recursive
 *
 * @tparam head
 * @tparam tail
 */
template <bool head, bool... tail>
struct variadic_and {
  static constexpr bool value = head && variadic_and<tail...>::value;
};

template <bool head>
struct variadic_and<head> {
  static constexpr bool value = head;
};

/**
 * @brief
 *
 * @tparam T
 */
template <typename... T>
struct is_type_unique;

template <>
struct is_type_unique<> : std::true_type {};

template <typename T>
struct is_type_unique<T> : std::true_type {};

template <typename F, typename S, typename... T>
struct is_type_unique<F, S, T...> {
  static constexpr bool value = is_type_unique<T...>::value and not std::is_same<F, S>::value;
};

template <bool Cond, typename T>
struct const_if {
  using type = std::conditional_t<Cond, std::add_const_t<T>, T>;
};

template <class T, std::size_t N>
constexpr std::size_t size(T const (&)[N]) noexcept {
  return N;
}

template <typename T>
constexpr auto size(T const& t_obj) -> decltype(t_obj.size()) {
  return t_obj.size();
}

template <typename T, std::size_t N>
constexpr decltype(auto) data(T const (&t_obj)[N]) {
  return t_obj;
}

template <typename T>
constexpr auto data(T const& t_obj) -> decltype(t_obj.data()) {
  return t_obj.data();
}

}  // namespace tmr_mt_helper

namespace tmr_listener {

// clang-format off
template <typename T> class Variable;
template <typename T> struct Expression;
// clang-format on

namespace detail {

template <typename T>
static constexpr bool is_named_var = tmr_mt_helper::is_specialization_of<T, Variable>::value;

template <typename T>
static constexpr bool is_expression = tmr_mt_helper::is_specialization_of<T, Expression>::value;

/**
 * @brief Since the argument passed can be r-valued or named variable, to identify its true underlying type, this helper
 *        class is used.
 *
 * @tparam T
 * @tparam T2
 */
template <typename T, typename T2 = void>
struct RealType;

template <typename T>
struct RealType<T, std::enable_if_t<is_named_var<T> or is_expression<T>>> {
  using type = typename T::underlying_t;
};

template <typename T>
struct RealType<T, std::enable_if_t<!(is_named_var<T> or is_expression<T>)>> {
  using type = T;
};

}  // namespace detail
}  // namespace tmr_listener

#endif