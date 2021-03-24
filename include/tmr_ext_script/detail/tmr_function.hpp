#ifndef TMR_MOTION_FUNCTION_IMPL_HPP_
#define TMR_MOTION_FUNCTION_IMPL_HPP_

#include <boost/fusion/container/vector.hpp>
#include <boost/fusion/include/find.hpp>

#include "tmr_command.hpp"
#include "tmr_fundamental_type.hpp"
#include "tmr_fwd.hpp"
#include "tmr_utility/tmr_constexpr_string.hpp"
#include "tmr_utility/tmr_mt_helper.hpp"
#include "tmr_utility/tmr_stringifier.hpp"

namespace tmr_listener {
namespace detail {

/**
 * @brief This class represents the concept of function for TM external scripting language
 *
 * @tparam ArgTypes variadic template serves as arguments of the function
 */
template <typename... ArgTypes>
class Function {
 public:
  /**
   * @brief operator() for best syntax resemblance
   *
   * @param t_name  The name of the function
   * @param t_args  The real argument passed to the function
   * @return string of the function call itself
   */
  template <typename PrintPolicy>
  auto operator()(PrintPolicy const& t_printer, std::string const& t_name,
                  FundamentalType<ArgTypes> const&... t_args) const noexcept {
    return t_printer(t_name, boost::algorithm::join(std::vector<std::string>{t_args.to_str()...}, ","));
  }
};

/**
 * @brief This class represents the concept of overload function for TM external scripting language
 *
 * @tparam Tag        Tag that group Header and functions that it can use, this prevents user from appending command
 *                    that the Header cannot use
 * @tparam RetType    Return type of the function, since TM motion function overloaded functions have same return
 *                    type, RetType is placed here instead of each Function
 * @tparam Functions  Function signatures
 */
template <typename Tag, typename PrintPolicy, typename RetType, typename... Functions>
struct FunctionSet {
  static_assert(tmr_mt_helper::variadic_and<tmr_mt_helper::is_specialization_of<Functions, Function>::value...>::value,
                "template param must be specialization of type Function");
  static_assert(tmr_mt_helper::is_type_unique<Functions...>::value, "Function signatures should be unique.");

  using FunctorContainer = boost::fusion::vector<Functions...>;
  using EndType          = typename boost::fusion::result_of::end<FunctorContainer>::type;

  tmr_listener::detail::ConstString name_;

  explicit constexpr FunctionSet(tmr_listener::detail::ConstString const t_str) noexcept : name_{t_str} {}

  /**
   * @brief operator() for best syntax resemblance
   *
   * @tparam Args
   * @param t_arguments Function arguments
   * @return Command
   *
   * @note  We need to make sure that the argument match the syntax of the overload sets, this is done by
   *        finding the type of the functor formed by the argument passed in the overload sets. Hence the
   *        static_assert
   */
  template <typename... Args>
  auto operator()(Args const&... t_arguments) const {
    using namespace boost::fusion::result_of;
    using TargetFunctor = Function<typename tmr_listener::detail::RealType<Args>::type...>;
    using FindResult    = typename find<FunctorContainer, TargetFunctor>::type;

    static_assert(not std::is_same<FindResult, EndType>::value, "Function signature not match");

    constexpr TargetFunctor function_call;
    return Command<Tag>{function_call(PrintPolicy{}, this->name_.to_std_str(), t_arguments...)};
  }
};

/**
 * @brief useful typedef
 */
template <typename RetType, typename... Functions>
using TMSTCFuncSet = FunctionSet<tmr_listener::detail::TMSCTTag, MotionFnCallPrinter, RetType, Functions...>;

template <typename... Functions>
using TMSTAFuncSet = FunctionSet<tmr_listener::detail::TMSTATag, SubCmdCallPrinter, void, Functions...>;

}  // namespace detail
}  // namespace tmr_listener

namespace tmr_listener {
namespace detail {

/**
 * @brief
 */
struct MotionFnCallPrinter {
  auto operator()(std::string const& t_name, std::string const& t_input) const noexcept {
    return t_name + '(' + t_input + ')';
  }
};

struct SubCmdCallPrinter {
  auto operator()(std::string const& t_name, std::string const& t_input) const noexcept {
    return t_name + ',' + t_input;
  }
};

/**
 * @brief
 *
 * @tparam T
 */
template <typename T>
inline auto assemble_to_msg(std::vector<std::string> const& t_input) noexcept {
  return boost::algorithm::join(t_input, ",");
}

template <>
inline auto assemble_to_msg<tmr_listener::detail::TMSCTTag>(std::vector<std::string> const& t_input) noexcept {
  auto const id   = t_input.front();
  auto const data = std::vector<std::string>{std::next(t_input.begin()), t_input.end()};
  return id + ',' + boost::algorithm::join(data, "\r\n");
}

}  // namespace detail
}  // namespace tmr_listener

#endif