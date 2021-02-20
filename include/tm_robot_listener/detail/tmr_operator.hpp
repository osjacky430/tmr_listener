#ifndef TMR_OPERATOR_HPP__
#define TMR_OPERATOR_HPP__

#include "tmr_fwd.hpp"

#include <boost/xpressive/xpressive.hpp>
#include <string>

// @todo some expression doesn't accept r-value
#define TM_UNARY_OP_IS_POSTFIX_0
#define TM_UNARY_OP_IS_POSTFIX_1 , int

#define TM_UNARY_OP_APPLY_POSTFIX_0(VAL, tok) tok VAL
#define TM_UNARY_OP_APPLY_POSTFIX_1(VAL, tok) VAL tok

#define TM_UNARY_OP_COMBINE_POSTFIX_0(VAL, tok) (#tok + VAL())
#define TM_UNARY_OP_COMBINE_POSTFIX_1(VAL, tok) (VAL() + #tok)

#define TM_DEFINE_UNARY_OPERATOR(CLASS, tok, POST)                                                        \
  template <typename T>                                                                                   \
  auto operator tok(CLASS<T> const& t_e TM_UNARY_OP_IS_POSTFIX_##POST) noexcept {                         \
    auto const op_ret_type = []() {                                                                       \
      T a{};                                                                                              \
      return TM_UNARY_OP_APPLY_POSTFIX_##POST(a, tok);                                                    \
    };                                                                                                    \
    return Expression<decltype(op_ret_type())>{'(' + TM_UNARY_OP_COMBINE_POSTFIX_##POST(t_e, tok) + ')'}; \
  }

#define TM_DEFINE_BINARY_OPERATOR(CLASS, tok)                                                         \
  template <typename T, typename U>                                                                   \
  auto operator tok(CLASS<T> const& t_e, U const& t_u) noexcept {                                     \
    using stringifier = std::conditional_t<detail::is_expression<U> or detail::is_named_var<U>,       \
                                           detail::statement_to_string, value_to_string<U>>;          \
                                                                                                      \
    auto const op_ret_type = []() {                                                                   \
      T a{};                                                                                          \
      typename detail::RealType<U>::type b{};                                                         \
      return a tok b;                                                                                 \
    };                                                                                                \
    return Expression<decltype(op_ret_type())>{'(' + t_e() + #tok + stringifier{}(t_u) + ')'};        \
  }                                                                                                   \
                                                                                                      \
  template <typename T, typename U,                                                                   \
            std::enable_if_t<!(detail::is_expression<U> or detail::is_named_var<U>), bool> = true>    \
  auto operator tok(U const& t_u, CLASS<T> const& t_e) noexcept {                                     \
    auto const op_ret_type = []() {                                                                   \
      T a{};                                                                                          \
      U b{};                                                                                          \
      return b tok a;                                                                                 \
    };                                                                                                \
    return Expression<decltype(op_ret_type())>{'(' + value_to_string<U>{}(t_u) + #tok + t_e() + ')'}; \
  }

#define TM_DEFINE_OPERATORS(CLASS)       \
  TM_DEFINE_UNARY_OPERATOR(CLASS, ++, 1) \
  TM_DEFINE_UNARY_OPERATOR(CLASS, --, 1) \
  TM_DEFINE_UNARY_OPERATOR(CLASS, ++, 0) \
  TM_DEFINE_UNARY_OPERATOR(CLASS, --, 0) \
  TM_DEFINE_UNARY_OPERATOR(CLASS, +, 0)  \
  TM_DEFINE_UNARY_OPERATOR(CLASS, -, 0)  \
  TM_DEFINE_UNARY_OPERATOR(CLASS, ~, 0)  \
  TM_DEFINE_UNARY_OPERATOR(CLASS, !, 0)  \
  TM_DEFINE_BINARY_OPERATOR(CLASS, *);   \
  TM_DEFINE_BINARY_OPERATOR(CLASS, /);   \
  TM_DEFINE_BINARY_OPERATOR(CLASS, %);   \
  TM_DEFINE_BINARY_OPERATOR(CLASS, +);   \
  TM_DEFINE_BINARY_OPERATOR(CLASS, -);   \
  TM_DEFINE_BINARY_OPERATOR(CLASS, <<);  \
  TM_DEFINE_BINARY_OPERATOR(CLASS, >>);  \
  TM_DEFINE_BINARY_OPERATOR(CLASS, >);   \
  TM_DEFINE_BINARY_OPERATOR(CLASS, >=);  \
  TM_DEFINE_BINARY_OPERATOR(CLASS, <);   \
  TM_DEFINE_BINARY_OPERATOR(CLASS, <=);  \
  TM_DEFINE_BINARY_OPERATOR(CLASS, ==);  \
  TM_DEFINE_BINARY_OPERATOR(CLASS, !=);  \
  TM_DEFINE_BINARY_OPERATOR(CLASS, &);   \
  TM_DEFINE_BINARY_OPERATOR(CLASS, ^);   \
  TM_DEFINE_BINARY_OPERATOR(CLASS, |);   \
  TM_DEFINE_BINARY_OPERATOR(CLASS, &&);  \
  TM_DEFINE_BINARY_OPERATOR(CLASS, ||);  \
  TM_DEFINE_BINARY_OPERATOR(CLASS, +=);  \
  TM_DEFINE_BINARY_OPERATOR(CLASS, -=);  \
  TM_DEFINE_BINARY_OPERATOR(CLASS, *=);  \
  TM_DEFINE_BINARY_OPERATOR(CLASS, /=);  \
  TM_DEFINE_BINARY_OPERATOR(CLASS, %=);  \
  TM_DEFINE_BINARY_OPERATOR(CLASS, <<=); \
  TM_DEFINE_BINARY_OPERATOR(CLASS, >>=); \
  TM_DEFINE_BINARY_OPERATOR(CLASS, &=);  \
  TM_DEFINE_BINARY_OPERATOR(CLASS, ^=);  \
  TM_DEFINE_BINARY_OPERATOR(CLASS, |=);

namespace tm_robot_listener {
namespace detail {

struct statement_to_string {
  template <typename T>
  std::string operator()(Expression<T> const& t_expr) const noexcept {
    return t_expr();
  }

  template <typename T>
  std::string operator()(Variable<T> const& t_var) const noexcept {
    return t_var();
  }
};

inline boost::xpressive::sregex var_name_pattern() noexcept {
  static auto const var_name_regex = boost::xpressive::sregex::compile("^[a-zA-Z_][a-zA-Z0-9_]*$");
  return var_name_regex;
}

}  // namespace detail
}  // namespace tm_robot_listener

#endif