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
template <typename Tag>
class Command {
  template <typename FunctionSetTag, typename PrintPolicy, typename RetType, typename... Functions>
  friend class tmr_listener::detail::FunctionSet;

  std::string const name;
  explicit Command(std::string t_name) : name(std::move(t_name)) {}

 public:
  std::string get_cmd() const noexcept { return this->name; }
};

}  // namespace tmr_listener

#endif