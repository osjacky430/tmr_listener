#ifndef TMR_PARSER_HPP_
#define TMR_PARSER_HPP_

#include <boost/spirit/include/qi.hpp>

namespace tmr_listener {

template <typename T>
using ParseRule = boost::spirit::qi::rule<std::string::iterator, T(), boost::spirit::ascii::space_type>;

}

#endif