#ifndef TMR_VARIABLE_HPP_
#define TMR_VARIABLE_HPP_

#include "tm_robot_listener/detail/tmr_fundamental_type.hpp"
#include "tm_robot_listener/detail/tmr_fwd.hpp"
#include "tm_robot_listener/detail/tmr_mt_helper.hpp"
#include "tm_robot_listener/detail/tmr_operator.hpp"
#include "tm_robot_listener/detail/tmr_stringifier.hpp"

#include <boost/format.hpp>
#include <boost/fusion/include/at_key.hpp>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace tm_robot_listener {

template <typename T, /*typename U,*/ std::enable_if_t<not tmr_mt_helper::is_std_array<T>::value, bool> = true>
auto declare(Variable<T> const& t_var, T const& t_val);

/**
 * @brief The class represents the concept of expression, for more explanation on expression, see
 *        [here](https://en.cppreference.com/w/cpp/language/expressions). Expression is a sequence of operators and
 *        their operands, that specifies a computation. In tmr_listener, operands can be @ref Variable or r-value. This
 *        is used for message generation, see example
 *
 * @tparam T  Result type of the expression evaluation
 *
 * @todo make the constructor private, and only friend Variable
 *
 * @code{.cpp}
 *
 *    Variable<int> some_int{"some_int"};
 *    TMSCT << ID{"1"} << declare(some_int, 1) << End();  // generate variable declaration expression
 *    TMSCT << ID{"1"} << some_int++ << End();            // generate variable post increment expression
 *
 * @endcode
 */
template <typename T>
struct Expression {
  using underlying_t = T;

  std::string const value;  // @note this variable is marked as const as I didn't make the construction and variable
                            // private, make it at least unmodifiable
  auto operator()() const noexcept { return this->value; }
};

TM_DEFINE_OPERATORS(Expression);

/**
 * @brief The class represents the concept of named variable in TM external script language, for more information, refer
 *        to tm_expression_editor_and_listen_node_reference_manual_en
 *
 * @tparam T  Underlying type of the variable
 */
template <typename T>
class Variable {  // NOLINT
 private:
  std::string const name_;  // @note this variable is marked as const as I didn't make the construction and variable
                            // private, make it at least unmodifiable
 public:
  using underlying_t = T;

  explicit Variable(std::string t_name) noexcept : name_(std::move(t_name)) {}

  auto operator()() const noexcept { return this->name_; }

  /**
   * @brief operator= overloading for assignment expression
   *
   * @tparam Input type, must be convertible to the underlying type of the Variable
   * @param t_input Input value, needs to be lexical castable
   * @return Assignment expression
   */
  template <typename U>
  auto operator=(U const& t_input) const noexcept {  // NOLINT
    static_assert(std::is_convertible<typename detail::RealType<U>::type, T>::value,
                  "No known conversion from input type to Variable underlying type");
    using stringifier= std::conditional_t<detail::is_expression<U> or detail::is_named_var<U>,
                                           detail::statement_to_string, value_to_string<U>>;
    return Expression<T>{'(' + this->name_ + '=' + stringifier{}(t_input) + ')'};
  }

  /**
   * @brief operator= overloading that is default-deleted due to const member variable
   *
   * @param t_input input variable
   * @return Assignement expression
   */
  auto operator=(Variable<T> const& t_input) const noexcept {  // NOLINT
    return Expression<T>{'(' + this->name_ + '=' + t_input() + ')'};
  }
};

TM_DEFINE_OPERATORS(Variable);

template <typename S, typename T, typename U, typename V>
inline auto ternary_expr(T const& t_expr, U const& t_left, V const& t_right) noexcept {
  constexpr auto expr_require      = (detail::is_named_var<T> or detail::is_expression<T>);
  constexpr auto expr_type_require = std::is_same<typename T::underlying_t, bool>::value;
  static_assert(expr_require and expr_type_require, "ternary operator require a boolean statement");

  constexpr auto left_type_require   = (detail::is_named_var<U> or detail::is_expression<U>);
  constexpr auto left_u_type_require = std::is_convertible<typename U::underlying_t, S>::value;
  static_assert(left_type_require and left_u_type_require,
                "Expression or Variable must be convertible to the underlying type of the Variable");

  constexpr auto right_type_require   = (detail::is_named_var<V> or detail::is_expression<V>);
  constexpr auto right_u_type_require = std::is_convertible<typename V::underlying_t, S>::value;
  static_assert(right_type_require and right_u_type_require,
                "Expression or Variable must be convertible to the underlying type of the Variable");

  return Expression<S>{'(' + t_expr() + '?' + t_left() + ':' + t_right() + ')'};
}

/**
 * @brief This function creates Variable declaration expression, array version
 *
 * @tparam T  underlying type of the variable
 * @param t_var Variable to be declared
 * @param t_val Initial value of the variable
 * @return Expression of initialization
 */
template <typename T, /*typename U,*/ std::enable_if_t<tmr_mt_helper::is_std_array<T>::value, bool> = true>
inline auto declare(Variable<T> const& t_var, T const& t_val) {
  // static_assert(); U must be one of the following: T, or underlying type of U that is convertible to T
  if (not boost::xpressive::regex_match(t_var(), detail::var_name_pattern())) {
    throw std::invalid_argument{"bad variable name: " + t_var()};
  }

  using namespace boost::fusion;
  auto const formatted = boost::format("%s[] %s=%s") %
                         at_key<typename T::value_type>(motion_function::detail::get_type_decl_str()) % t_var() %
                         value_to_string<T>{}(t_val);
  return Expression<T>{formatted.str()};
}

/**
 * @brief This function creates Variable declaration expression, non-array version
 *
 * @tparam T  underlying type of the variable
 * @param t_var Variable to be declared
 * @param t_val Initial value of the variable
 * @return Expression of initialization
 *
 * @todo Add case for Variable and Expression
 */
template <typename T, /*typename U,*/ std::enable_if_t<not tmr_mt_helper::is_std_array<T>::value, bool> = true>
inline auto declare(Variable<T> const& t_var, T const& t_val) {
  // static_assert();
  if (not boost::xpressive::regex_match(t_var(), detail::var_name_pattern())) {
    throw std::invalid_argument{"bad variable name: " + t_var()};
  }

  using namespace boost::fusion;
  auto const formatted = boost::format("%s %s=%s") % at_key<T>(motion_function::detail::get_type_decl_str()) % t_var() %
                         value_to_string<T>{}(t_val);
  return Expression<T>{formatted.str()};
}

/**
 * @brief r-value Variable is deleted
 */
template <typename T>
inline auto declare(Variable<T>&&, T const&) = delete;

}  // namespace tm_robot_listener

#endif