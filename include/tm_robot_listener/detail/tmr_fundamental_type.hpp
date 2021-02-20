#ifndef TMR_FUNDAMENTAL_TYPE_HPP__
#define TMR_FUNDAMENTAL_TYPE_HPP__

#include <boost/fusion/container/map.hpp>
#include <boost/variant.hpp>
#include <string>

#include "tmr_fwd.hpp"
#include "tmr_stringifier.hpp"

namespace tm_robot_listener {
namespace motion_function {
namespace detail {
/**
 * @brief A light wrapper to represent named variable and literal constant in tm external script language
 *
 * @tparam T  Underlying type
 *
 * @note  Design requirement:
 *
 *          Suppose we have a function that takes one argument:
 *
 *              bool some_func(FundamentalType<int> const&);
 *
 *          FundamentalType<T> must be designed in the way such that it can be called by:
 *
 *              int some_int = 1;
 *              some_func(some_int);    // must pass
 *              some_func(1);           // must pass
 *
 *              Variable<int> v{"v"};
 *              some_func(v);           // must pass
 *
 *              some_func(Variable<int>{"V", 2.0})  // special case, not sure how to deal with it atm
 */
template <typename T>
class FundamentalType {
 public:
  using operating_t  = boost::variant<T, Variable<T>>;
  using underlying_t = T;

  constexpr FundamentalType() = default;
  constexpr FundamentalType(operating_t const& t_val) : val_(t_val) {}
  constexpr FundamentalType(T const& t_val) : val_(t_val) {}
  constexpr FundamentalType(Variable<T> const& t_val) : val_(t_val) {}

  std::string to_str() const noexcept {
    if (auto const val = boost::get<Variable<T>>(&this->val_)) {
      return (*val)();
    }

    auto const val = boost::get<T>(&this->val_);
    return value_to_string<T>{}(*val);
  }

 private:
  operating_t val_;
};

}  // namespace detail
}  // namespace motion_function
}  // namespace tm_robot_listener

namespace tm_robot_listener {
namespace motion_function {
namespace detail {

/**
 * @brief
 *
 * @tparam Type
 */
template <typename Type>
using TypeNamePair = boost::fusion::pair<Type, std::string>;

using TypeMap = boost::fusion::map<TypeNamePair<int>, TypeNamePair<double>, TypeNamePair<bool>, TypeNamePair<float>,
                                   TypeNamePair<std::string>, TypeNamePair<std::uint8_t>, TypeNamePair<std::int16_t>,
                                   TypeNamePair<std::int32_t>>;

static auto const TYPE_STRING_MAP = TypeMap{
  boost::fusion::make_pair<int>("int"),
  boost::fusion::make_pair<double>("double"),
  boost::fusion::make_pair<bool>("bool"),
  boost::fusion::make_pair<float>("float"),
  boost::fusion::make_pair<std::string>("string"),
  boost::fusion::make_pair<std::uint8_t>("byte"),
  boost::fusion::make_pair<std::int16_t>("int16"),
  boost::fusion::make_pair<std::int32_t>("int32"),
};

}  // namespace detail
}  // namespace motion_function
}  // namespace tm_robot_listener

#endif