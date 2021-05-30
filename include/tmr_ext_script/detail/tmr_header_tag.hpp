#ifndef TMR_HEADER_TAG_HPP_
#define TMR_HEADER_TAG_HPP_

#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/spirit/include/qi.hpp>

#include <string>

namespace tmr_listener {
namespace detail {

template <typename T>
using ParseRule = boost::spirit::qi::rule<std::string::iterator, T(), boost::spirit::ascii::space_type>;

/**
 * @brief Utility class for Header usage classification
 */
struct TMSCTTag {
  static constexpr auto HEADER() noexcept { return "$TMSCT"; }

  struct DataFormat {
    struct FunctionCall;
    struct VariableDecl;

    using AvailableCmd = boost::variant<FunctionCall, VariableDecl>;
    using Data         = std::vector<AvailableCmd>;

    std::string id_;
    Data cmd_;

    static ParseRule<DataFormat>& parsing_rule();
  };
};

/**
 * @brief Utility class for Header usage classification
 */
struct TMSTATag {
  static constexpr auto HEADER() noexcept { return "$TMSTA"; }

  struct DataFormat {
    std::string cmd_;

    static ParseRule<TMSTATag::DataFormat>& parsing_rule();
  };
};

/**
 * @brief Utility class for Header usage classification
 */
struct CPERRTag {
  static constexpr auto HEADER() noexcept { return "$CPERR"; }
};

}  // namespace detail
}  // namespace tmr_listener

BOOST_FUSION_ADAPT_STRUCT(tmr_listener::detail::TMSCTTag::DataFormat, id_, cmd_)
BOOST_FUSION_ADAPT_STRUCT(tmr_listener::detail::TMSTATag::DataFormat, cmd_)

#endif