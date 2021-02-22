#ifndef TMR_MOTION_FUNCTION_IMPL__
#define TMR_MOTION_FUNCTION_IMPL__

#include <boost/fusion/container/vector.hpp>
#include <boost/fusion/include/find.hpp>

#include "tmr_command.hpp"
#include "tmr_fundamental_type.hpp"
#include "tmr_fwd.hpp"
#include "tmr_mt_helper.hpp"
#include "tmr_stringifier.hpp"

namespace tm_robot_listener {
namespace motion_function {
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
class FunctionSet {
  static_assert(tmr_mt_helper::variadic_and<tmr_mt_helper::is_specialization_of<Functions, Function>::value...>::value,
                "template param must be specialization of type Function");
  static_assert(tmr_mt_helper::is_type_unique<Functions...>::value, "Function signatures should be unique.");

  char const* const name_;
  std::size_t const size_;

 public:
  using FunctorContainer = boost::fusion::vector<Functions...>;
  using EndType          = typename boost::fusion::result_of::end<FunctorContainer>::type;

  template <std::size_t N>
  explicit constexpr FunctionSet(char const (&t_name)[N]) noexcept
    : name_(static_cast<char const* const>(t_name)), size_{N} {
    static_assert(N != 0, "invalid name");
  }

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
    using TargetFunctor = Function<typename tm_robot_listener::detail::RealType<Args>::type...>;
    using FindResult    = typename find<FunctorContainer, TargetFunctor>::type;

    static_assert(not std::is_same<FindResult, EndType>::value, "Function signature not match");

    TargetFunctor const function_call;
    return Command<Tag>{function_call(PrintPolicy{}, std::string{this->name_}, t_arguments...)};
  }
};

/**
 * @brief Utility class for Header usage classification
 */
struct TMSCTTag {
  static constexpr auto HEADER() { return "$TMSCT"; }
};

/**
 * @brief Utility class for Header usage classification
 */
struct TMSTATag {
  static constexpr auto HEADER() { return "$TMSTA"; }
};

/**
 * @brief Utility class for Header usage classification
 */
struct CPERRTag {
  static constexpr auto HEADER() { return "$CPERR"; }
};

/**
 * @brief useful typedef
 */
template <typename RetType, typename... Functions>
using TMSTCFuncSet = FunctionSet<TMSCTTag, MotionFnCallPrinter, RetType, Functions...>;

template <typename... Functions>
using TMSTAFuncSet = FunctionSet<TMSTATag, SubCmdCallPrinter, void, Functions...>;

}  // namespace detail
}  // namespace motion_function
}  // namespace tm_robot_listener

namespace tm_robot_listener {
namespace motion_function {
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
struct assemble_to_msg {
  auto operator()(std::vector<std::string> const& t_input) const noexcept {
    using namespace boost::algorithm;

    return join(t_input, ",");
  }
};

template <>
struct assemble_to_msg<TMSCTTag> {
  auto operator()(std::vector<std::string> const& t_input) const noexcept {
    using namespace boost::algorithm;

    auto const id   = t_input.front();
    auto const data = std::vector<std::string>{std::next(t_input.begin()), t_input.end()};
    return id + ',' + join(data, "\r\n");
  }
};

}  // namespace detail
}  // namespace motion_function
}  // namespace tm_robot_listener

#endif