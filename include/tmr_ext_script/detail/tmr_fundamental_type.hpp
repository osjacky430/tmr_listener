#ifndef TMR_FUNDAMENTAL_TYPE_HPP_
#define TMR_FUNDAMENTAL_TYPE_HPP_

#include <boost/fusion/container/map.hpp>
#include <boost/variant.hpp>
#include <string>

#include "tmr_fwd.hpp"
#include "tmr_utility/tmr_constexpr_string.hpp"
#include "tmr_utility/tmr_stringifier.hpp"

namespace tmr_listener {
namespace detail {
/**
 * @brief A light wrapper to represent named variable and literal constant in tm external script language
 *
 * @tparam T  Underlying type
 *
 * @note  Design requirement:
 *
 *          Suppose we have a function that takes one argument (this is actually what we are dealing with, e.g., we can
 *          call tm motion function with both literal constants, or variables declared in TM script interpreter):
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
 *              some_func(Variable<int>{"V"})  // special case, not sure how to deal with it atm
 *
 *        That said, the constructor should not be marked explicit cause implicit conversion is intended.
 *
 * @todo Rethink if I need this or not, this to me looks like some stringifier that distinct how to stringify value in
 *       run time. (it can and should be done at compile time?)
 */
template <typename T>
class FundamentalType {
 public:
  using operating_t  = boost::variant<T, Variable<T>>;
  using underlying_t = T;

  constexpr FundamentalType() = default;
  constexpr FundamentalType(operating_t const& t_val) : val_(t_val) {}  // NOLINT
  constexpr FundamentalType(T const& t_val) : val_(t_val) {}            // NOLINT
  constexpr FundamentalType(Variable<T> const& t_val) : val_(t_val) {}  // NOLINT

  std::string to_str() const noexcept {
    if (auto const val = boost::get<Variable<T>>(&this->val_)) {
      return val->to_str();
    }

    auto const val = boost::get<T>(&this->val_);
    return value_to_string<T>{}(*val);
  }

 private:
  operating_t val_;
};

}  // namespace detail
}  // namespace tmr_listener

namespace tmr_listener {
namespace detail {

/**
 * @brief
 *
 * @tparam Type
 */
template <typename Type>
using TypeNamePair = boost::fusion::pair<Type, tmr_listener::detail::ConstString>;

using TypeMap = boost::fusion::map<TypeNamePair<int>, TypeNamePair<double>, TypeNamePair<bool>, TypeNamePair<float>,
                                   TypeNamePair<std::string>, TypeNamePair<std::uint8_t>, TypeNamePair<std::int16_t>,
                                   TypeNamePair<std::int32_t>>;

/**
 * @brief Get the type decl str object
 *
 * @return ConstString
 * @note  This is meant to be called at compile time, do not call it at run time. It is run time inefficient
 */
template <typename T>
inline auto constexpr get_type_decl_str() noexcept {
  constexpr auto INT_DECL      = TypeNamePair<int>{"int"};
  constexpr auto DOUBLE_DECL   = TypeNamePair<double>{"double"};
  constexpr auto BOOL_DECL     = TypeNamePair<bool>{"bool"};
  constexpr auto FLOAT_DECL    = TypeNamePair<float>{"float"};
  constexpr auto STRING_DECL   = TypeNamePair<std::string>{"string"};
  constexpr auto BYTE_DECL     = TypeNamePair<std::uint8_t>{"byte"};
  constexpr auto HALFWORD_DECL = TypeNamePair<std::int16_t>{"int16"};
  constexpr auto WORD_DECL     = TypeNamePair<std::int32_t>{"int32"};

  constexpr auto type_map = TypeMap{
    INT_DECL, DOUBLE_DECL, BOOL_DECL, FLOAT_DECL, STRING_DECL, BYTE_DECL, HALFWORD_DECL, WORD_DECL,
  };
  return boost::fusion::at_key<T>(type_map);
}

}  // namespace detail
}  // namespace tmr_listener

#endif