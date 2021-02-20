#ifndef TMR_STRINGIFIER_HPP__
#define TMR_STRINGIFIER_HPP__

#include <array>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/range/adaptors.hpp>
#include <string>

namespace boost {

template <>
inline std::string lexical_cast<std::string, bool>(bool const& t_bool) {
  return t_bool ? "true" : "false";
}

template <>
inline std::string lexical_cast<std::string, std::string>(std::string const& t_str) {
  return '\"' + t_str + '\"';
}

}  // namespace boost

namespace tm_robot_listener {

template <typename T>
static constexpr auto lexical_cast_string = boost::lexical_cast<std::string, T>;

template <typename T>
struct value_to_string {
  std::string operator()(T const& t_in) const noexcept { return boost::lexical_cast<std::string>(t_in); }
};

template <typename T, std::size_t N>
struct value_to_string<std::array<T, N>> {
  std::string operator()(std::array<T, N> const& t_in) const noexcept {
    using namespace boost::algorithm;
    using namespace boost::adaptors;

    return '{' + join(t_in | transformed(lexical_cast_string<T>), ",") + '}';
  }
};

}  // namespace tm_robot_listener

#endif