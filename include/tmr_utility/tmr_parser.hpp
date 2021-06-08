#ifndef TMR_PARSER_HPP_
#define TMR_PARSER_HPP_

#include <boost/fusion/container/map.hpp>
#include <boost/phoenix.hpp>
#include <boost/spirit/include/qi.hpp>

#include <array>
#include <string>
#include <vector>

namespace tmr_listener {
namespace detail {

static inline auto get_tm_type_parser() {
  using namespace boost::spirit;
  using namespace boost::fusion;
  using ParseTypeMap = map<pair<int, int_type>, pair<float, float_type>, pair<double, double_type>,
                           pair<bool, bool_type>, pair<std::uint8_t, qi::uint_parser<std::uint8_t>>>;
  static const ParseTypeMap type_map{make_pair<int>(int_), make_pair<float>(float_), make_pair<double>(double_),
                                     make_pair<bool>(bool_), make_pair<std::uint8_t>(qi::uint_parser<std::uint8_t>{})};

  return type_map;
}

}  // namespace detail

template <typename T>
using ParseRule = boost::spirit::qi::rule<std::string::iterator, T(), boost::spirit::ascii::space_type>;

template <typename T>
static inline T parse_as(std::string const& t_str_to_parse) {
  using namespace boost::spirit;
  using boost::fusion::at_key;

  T ret_val;
  auto const& type_map = detail::get_tm_type_parser();
  bool const result    = qi::parse(t_str_to_parse.cbegin(), t_str_to_parse.cend(), at_key<T>(type_map), ret_val);
  if (not result) {
    throw std::invalid_argument("bad input");
  }

  return ret_val;
}

template <typename T, std::size_t N>
static inline std::array<T, N> parse_as(std::string const& t_str_to_parse) {
  using namespace boost::spirit;
  using namespace boost::phoenix;
  using boost::fusion::at_key;

  int i = 0;
  std::array<T, N> ret_val;
  auto const& type_map = detail::get_tm_type_parser();
  qi::parse(t_str_to_parse.cbegin(), t_str_to_parse.cend(),
            ('[' >> at_key<T>(type_map)[at(boost::phoenix::ref(ret_val), ref(i)++) = boost::spirit::_1] % ',' >> ']'));
  return ret_val;
}

}  // namespace tmr_listener

#endif