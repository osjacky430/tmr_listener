#include "tmr_ext_script/detail/tmr_header_tag.hpp"

#include <boost/fusion/include/boost_tuple.hpp>

using boost::phoenix::at_c;
using boost::phoenix::static_cast_;
using boost::spirit::as_string;
using boost::spirit::ascii::alpha;
using boost::spirit::ascii::space;
using boost::spirit::qi::_val;
using boost::spirit::qi::bool_;
using boost::spirit::qi::char_;
using boost::spirit::qi::eol;
using boost::spirit::qi::int_;
using boost::spirit::qi::int_parser;
using boost::spirit::qi::lexeme;
using boost::spirit::qi::symbols;

namespace tmr_listener {
namespace detail {

ParseRule<TMSCTTag::DataFormat::FunctionCall>& TMSCTTag::DataFormat::FunctionCall::parsing_rule() noexcept {
  static ParseRule<std::string> const function_name = as_string[+alpha];
  static ParseRule<std::string> const arguments     = as_string[*(char_ - ")")];

  static ParseRule<TMSCTTag::DataFormat::FunctionCall> rule = function_name >> "(" >> arguments >> ")";
  return rule;
}

ParseRule<TMSCTTag::DataFormat::VariableDecl>& TMSCTTag::DataFormat::VariableDecl::parsing_rule() noexcept {
  static ParseRule<std::string> const type_rule = as_string[lexeme[+(char_ - space)]];
  static ParseRule<std::string> const name_rule = as_string[+(char_ - space - '=')];
  static ParseRule<std::string> const val_rule  = as_string[lexeme[+(char_ - eol - ",*")]];

  static ParseRule<TMSCTTag::DataFormat::VariableDecl> rule = type_rule >> name_rule >> '=' >> val_rule;
  return rule;
}

ParseRule<TMSCTTag::DataFormat::ScriptResult>& TMSCTTag::DataFormat::ScriptResult::parsing_rule() noexcept {
  static symbols<char, bool> const result_msg{std::array<std::string, 2>{"OK", "ERROR"},
                                              std::array<bool, 2>{true, false}};
  static ParseRule<std::vector<int>> const abnormal_lines_ = ";" >> (int_ % ";");

  static ParseRule<ScriptResult> parser = (result_msg >> -abnormal_lines_) - ",*";
  return parser;
}

ParseRule<TMSCTTag::DataFormat>& TMSCTTag::DataFormat::parsing_rule() {
  static ParseRule<std::string> const id_rule        = +(char_ - ",");
  static ParseRule<ClientRequest> const client_rule  = +(FunctionCall::parsing_rule() | VariableDecl::parsing_rule());
  static ParseRule<ServerResponse> const server_rule = ScriptResult::parsing_rule() | *(char_ - ",*");
  static ParseRule<Data> const data_rule             = client_rule | server_rule;

  static ParseRule<DataFormat> rule = id_rule >> "," >> data_rule;
  return rule;
}

ParseRule<TMSTATag::DataFormat>& TMSTATag::DataFormat::parsing_rule() {
  using namespace boost::spirit;

  static symbols<char, int> const stat_map{std::array<std::string, 3>{"true", "false", "none"},
                                           std::array<int, 3>{0, 1, 2}};
  static ParseRule<Subcmd00Resp> rule_00_ = "00," >> bool_ >> "," >> *(char_ - ",*");
  static ParseRule<std::string> cmd       = +(char_ - ",");
  static ParseRule<Subcmd01Resp> rule_01_ = "01," >> cmd[at_c<0>(_val) = _1] >> ","  //
                                            >> stat_map[at_c<1>(_val) = static_cast_<TagNumberStatus>(qi::_1)];

  static ParseRule<DataFormat> rule = rule_00_ | rule_01_;
  return rule;
}

ParseRule<CPERRTag::DataFormat>& CPERRTag::DataFormat::parsing_rule() noexcept {
  using namespace boost::spirit;
  static ParseRule<ErrorCode> const err_code_rule = int_parser<int, 16>{}[_val = static_cast_<ErrorCode>(qi::_1)];
  static ParseRule<DataFormat> response_parser    = err_code_rule - ",*";
  return response_parser;
}

}  // namespace detail
}  // namespace tmr_listener

using namespace tmr_listener::detail;

BOOST_FUSION_ADAPT_STRUCT(TMSCTTag::DataFormat::FunctionCall, name_, args_)
BOOST_FUSION_ADAPT_STRUCT(TMSCTTag::DataFormat::VariableDecl, type_, name_, val_)
BOOST_FUSION_ADAPT_STRUCT(TMSCTTag::DataFormat::ScriptResult, result_, abnormal_lines_)