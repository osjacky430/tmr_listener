#include "tmr_ext_script/detail/tmr_header_tag.hpp"

using boost::phoenix::at_c;
using boost::phoenix::static_cast_;
using boost::spirit::qi::_1;
using boost::spirit::qi::_val;
using boost::spirit::qi::bool_;
using boost::spirit::qi::char_;
using boost::spirit::qi::int_;
using boost::spirit::qi::int_parser;
using boost::spirit::qi::lexeme;
using boost::spirit::qi::symbols;

namespace tmr_listener {
namespace detail {

// @todo id rule doesn't belong to here
static ParseRule<TMSCTEnterNodeMsg>& tmsct_enter_node_msg_rule() noexcept {
  static ParseRule<std::string> const id_rule        = +(char_ - ",");
  static ParseRule<std::string> const msg_rule       = *(char_ - ",*");
  static ParseRule<TMSCTEnterNodeMsg> enter_node_msg = id_rule >> "," >> msg_rule;
  return enter_node_msg;
}

// @todo id rule doesn't belong to here
static ParseRule<TMSCTResponse>& tmsct_response_rule() noexcept {
  static ParseRule<std::string> const id_rule = +(char_ - ",");
  static symbols<char, bool> const result_msg{std::array<std::string, 2>{"OK", "ERROR"},
                                              std::array<bool, 2>{true, false}};
  static ParseRule<std::vector<int>> const abnormal_lines_ = ";" >> (int_ % ";");

  static ParseRule<TMSCTResponse> parser = (id_rule >> "," >> result_msg >> -abnormal_lines_) - ",*";
  return parser;
}

// @todo id rule belongs to here
ParseRule<TMSCTTag::DataFrame>& TMSCTTag::DataFrame::parsing_rule() {
  static ParseRule<DataFrame> rule = tmsct_response_rule() | tmsct_enter_node_msg_rule();
  return rule;
}

ParseRule<TMSTATag::DataFrame>& TMSTATag::DataFrame::parsing_rule() {
  static symbols<char, int> const stat_map{std::array<std::string, 3>{"true", "false", "none"},
                                           std::array<int, 3>{0, 1, 2}};
  static ParseRule<std::string> const cmd = +(char_ - ",");

  static ParseRule<TMSTAResponse::Subcmd00> const rule_00_ = "00," >> bool_ >> "," >> *(char_ - ",*");
  static ParseRule<TMSTAResponse::Subcmd01> const rule_01_ =
    "01," >> cmd[at_c<0>(_val) = _1] >> "," >> stat_map[at_c<1>(_val) = static_cast_<TagNumberStatus>(_1)];
  static ParseRule<TMSTAResponse::DataMsg> const rule_90_99_ = int_ >> "," >> lexeme[+(char_ - ",*")];

  static ParseRule<DataFrame> rule = rule_00_ | rule_01_ | rule_90_99_;
  return rule;
}

ParseRule<CPERRTag::DataFrame>& CPERRTag::DataFrame::parsing_rule() noexcept {
  static ParseRule<ErrorCode> const err_code_rule = int_parser<int, 16>{}[_val = static_cast_<ErrorCode>(_1)];
  static ParseRule<CPERRResponse> const cperr     = err_code_rule;
  static ParseRule<DataFrame> response_parser     = cperr - ",*";
  return response_parser;
}

}  // namespace detail
}  // namespace tmr_listener

using namespace tmr_listener;

BOOST_FUSION_ADAPT_STRUCT(TMSTAResponse::Subcmd00, entered_, node_name_)
BOOST_FUSION_ADAPT_STRUCT(TMSTAResponse::Subcmd01, tag_number_, tag_stat_)
BOOST_FUSION_ADAPT_STRUCT(TMSTAResponse::DataMsg, cmd_, data_)
BOOST_FUSION_ADAPT_STRUCT(TMSCTEnterNodeMsg, id_, msg_)
BOOST_FUSION_ADAPT_STRUCT(TMSCTResponse, id_, script_result_, abnormal_lines_)
BOOST_FUSION_ADAPT_STRUCT(CPERRResponse, err_)