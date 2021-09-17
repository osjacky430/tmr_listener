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

/**
 * @brief This function parse input string as type specified in template parameter T. it will throw if it failed to
 *        parse the input string to the desired type
 *
 * @tparam T  The type of the content of input string
 * @param t_str_to_parse  Input string contains desire type
 * @return T  Content of t_str_to_parse
 */
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

/**
 * @brief This function parses input string as array, it will throw if the number of elements in the input
 *        string doesn't match the number of NTTP, i.e. N.
 *
 * @tparam T  The type of the array element
 * @tparam N  Number of elements, since TM has only fix sized array, this must be known at compile time
 * @param t_str_to_parse  Input string contains array
 * @return std::array<T, N> Content of t_str_to_parse
 */
template <typename T, std::size_t N>
static inline std::array<T, N> parse_as(std::string const& t_str_to_parse) {
  using namespace boost::spirit;
  using namespace boost::phoenix;
  using boost::fusion::at_key;

  std::size_t i = 0;
  std::array<T, N> ret_val;
  auto const& type_map = detail::get_tm_type_parser();

  auto const str_to_arr = qi::copy(at_key<T>(type_map)[at(boost::phoenix::ref(ret_val), ref(i)++) = qi::_1] % ',');
  auto const squared_bracket_enclosed_arr = qi::copy('[' >> str_to_arr >> ']');
  auto const curly_bracket_enclosed_arr   = qi::copy('{' >> str_to_arr >> '}');
  qi::parse(t_str_to_parse.cbegin(), t_str_to_parse.cend(), squared_bracket_enclosed_arr | curly_bracket_enclosed_arr);

  if (N > i) {
    using namespace std::string_literals;
    throw std::invalid_argument("Result contains "s + std::to_string(i) + " elements, which differs from expected: "s +
                                std::to_string(N));
  }
  // @todo utilize return value of parse ? or simply compare the local variable with the size of array
  return ret_val;
}

}  // namespace tmr_listener

#endif