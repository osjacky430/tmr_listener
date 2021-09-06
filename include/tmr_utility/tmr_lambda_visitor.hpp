#ifndef TMR_LAMBDA_VISITOR_HPP_
#define TMR_LAMBDA_VISITOR_HPP_

#include <boost/variant.hpp>
#include <type_traits>

#include "tmr_mt_helper.hpp"

namespace tmr_listener {

template <typename... Lambda>
struct LambdaVisitor;

template <typename Lambda1, typename... RestOfLambda>
struct LambdaVisitor<Lambda1, RestOfLambda...> : public LambdaVisitor<RestOfLambda...>, public Lambda1 {
  using Lambda1::operator();
  using LambdaVisitor<RestOfLambda...>::operator();
  LambdaVisitor(Lambda1 t_lambda_1, RestOfLambda... t_rest_of_lambdas) noexcept
    : LambdaVisitor<RestOfLambda...>(t_rest_of_lambdas...), Lambda1(t_lambda_1) {}
};

// according to reference collapsing rule, Lambda1 won't be a reference
template <typename Lambda1, typename... RestOfLambda>
struct LambdaVisitor<Lambda1&, RestOfLambda...> : public LambdaVisitor<RestOfLambda...>, public Lambda1 {
  using Lambda1::operator();
  using LambdaVisitor<RestOfLambda...>::operator();
  LambdaVisitor(Lambda1 t_lambda_1, RestOfLambda... t_rest_of_lambdas) noexcept
    : LambdaVisitor<RestOfLambda...>(t_rest_of_lambdas...), Lambda1(t_lambda_1) {}
};

template <typename Lambda1>
struct LambdaVisitor<Lambda1> : public boost::static_visitor<>, public Lambda1 {
  using Lambda1::operator();
  LambdaVisitor(Lambda1 t_lambda_1) noexcept : boost::static_visitor<>(), Lambda1(t_lambda_1) {}
};

template <typename Lambda1>
struct LambdaVisitor<Lambda1&> : public boost::static_visitor<>, public Lambda1 {
  using Lambda1::operator();
  LambdaVisitor(Lambda1 t_lambda_1) noexcept : boost::static_visitor<>(), Lambda1(t_lambda_1) {}
};

template <typename... Lambdas>
LambdaVisitor<Lambdas...> make_lambda_visitor(Lambdas&&... lambdas) {
  return {std::forward<Lambdas>(lambdas)...};
}

template <typename T, std::enable_if_t<tmr_mt_helper::is_specialization_of<std::remove_cv_t<T>, boost::variant>::value,
                                       bool> = true>
decltype(auto) as_variant(T& t_to_wrap) noexcept {
  return t_to_wrap;
}

template <
  typename T,
  std::enable_if_t<not tmr_mt_helper::is_specialization_of<std::remove_cv_t<T>, boost::variant>::value, bool> = true>
auto as_variant(T& t_to_wrap) noexcept {
  return boost::variant<std::remove_cv_t<T>>{t_to_wrap};
}

}  // namespace tmr_listener

#endif