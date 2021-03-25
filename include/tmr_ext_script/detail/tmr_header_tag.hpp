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
  static constexpr auto HEADER() { return "$TMSCT"; }

  struct DataFormat {
    struct FunctionCall {
      std::string name_;
      std::string args_;

      static auto& parsing_rule() noexcept {
        using boost::spirit::as_string;
        using boost::spirit::ascii::alpha;
        using boost::spirit::qi::char_;

        static ParseRule<std::string> const function_name = as_string[+alpha];
        static ParseRule<std::string> const arguments     = as_string[*(char_ - ")")];

        static ParseRule<FunctionCall> rule = function_name >> "(" >> arguments >> ")";
        return rule;
      }
    };

    struct VariableDecl {
      std::string type_;
      std::string name_;
      std::string val_;

      static auto& parsing_rule() noexcept {
        using boost::spirit::as_string;
        using boost::spirit::ascii::space;
        using boost::spirit::qi::char_;
        using boost::spirit::qi::eol;
        using boost::spirit::qi::lexeme;

        static ParseRule<std::string> const type_rule = as_string[lexeme[+(char_ - space)]];
        static ParseRule<std::string> const name_rule = as_string[+(char_ - space - '=')];
        static ParseRule<std::string> const val_rule  = as_string[lexeme[+(char_ - eol - ",*")]];

        static ParseRule<VariableDecl> rule = type_rule >> name_rule >> '=' >> val_rule;
        return rule;
      }
    };

    using AvailableCmd = boost::variant<FunctionCall, VariableDecl>;
    using Data         = std::vector<AvailableCmd>;

    std::string id_;
    Data cmd_;

    static auto& parsing_rule() {
      using boost::spirit::qi::char_;
      using boost::spirit::qi::int_;
      using boost::spirit::qi::lexeme;
      using boost::spirit::qi::lit;

      static ParseRule<std::string> const id_rule = +(char_ - ",");
      static ParseRule<Data> const data_rule      = *(FunctionCall::parsing_rule() | VariableDecl::parsing_rule());

      static ParseRule<DataFormat> rule = id_rule >> "," >> data_rule;
      return rule;
    }
  };
};

/**
 * @brief Utility class for Header usage classification
 */
struct TMSTATag {
  static constexpr auto HEADER() { return "$TMSTA"; }

  struct DataFormat {
    std::string cmd_;

    static auto& parsing_rule() {
      using boost::spirit::as_string;
      using boost::spirit::qi::char_;
      using boost::spirit::qi::int_;
      using boost::spirit::qi::lit;

      static ParseRule<DataFormat> rule = as_string[+(char_ - ",")];
      return rule;
    }
  };
};

/**
 * @brief Utility class for Header usage classification
 */
struct CPERRTag {
  static constexpr auto HEADER() { return "$CPERR"; }
};

}  // namespace detail
}  // namespace tmr_listener

BOOST_FUSION_ADAPT_STRUCT(tmr_listener::detail::TMSCTTag::DataFormat, id_, cmd_)
BOOST_FUSION_ADAPT_STRUCT(tmr_listener::detail::TMSCTTag::DataFormat::FunctionCall, name_, args_)
BOOST_FUSION_ADAPT_STRUCT(tmr_listener::detail::TMSCTTag::DataFormat::VariableDecl, type_, name_, val_)
BOOST_FUSION_ADAPT_STRUCT(tmr_listener::detail::TMSTATag::DataFormat, cmd_)

#endif