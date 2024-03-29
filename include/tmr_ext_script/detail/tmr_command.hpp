#ifndef TMR_COMMAND_HPP_
#define TMR_COMMAND_HPP_

#include <string>

#include "tmr_fwd.hpp"

namespace tmr_listener {

/**
 * @brief This class serves as a strong type of string
 *
 * @tparam Tag  Tag that group Header and function that can be used, this prevents user from appending command
 *              that the Header cannot use
 */
template <typename Tag, typename UType = void>
class Command {
  template <typename FunctionSetTag, typename PrintPolicy, typename RetType, typename... Functions>
  friend struct tmr_listener::detail::FunctionSet;

  std::string const value;
  explicit Command(std::string t_name) : value(std::move(t_name)) {}

 public:
  using underlying_t = UType;

  std::string to_str() const noexcept { return this->value; }
  auto as_expression() const noexcept { return Expression<UType>{this->value}; }
};

}  // namespace tmr_listener

#endif