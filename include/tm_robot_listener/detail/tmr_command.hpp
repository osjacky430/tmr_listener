#ifndef TMR_COMMAND_HPP__
#define TMR_COMMAND_HPP__

#include <string>

namespace tm_robot_listener {

/**
 * @brief This class serves as a strong type of string
 *
 * @tparam Tag  Tag that group Header and function that can be used, this prevents user from appending command
 *              that the Header cannot use
 *
 * @todo  We need to use friend class so only certain class have access to it
 */
template <typename Tag>
struct Command {
  std::string name;

  explicit Command(std::string t_name) : name(std::move(t_name)) {}

  std::string get_cmd() const noexcept { return this->name; }
};

}  // namespace tm_robot_listener

#endif