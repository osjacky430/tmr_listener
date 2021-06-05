#include <functional>

#include <boost/algorithm/algorithm.hpp>
#include <boost/phoenix.hpp>

#include "tmr_ethernet/detail/tmr_header_tag.hpp"
#include "tmr_ethernet/tmr_eth_rw.hpp"

namespace tmr_listener {

ParseRule<TMSVRJsonData>& TMSVRJsonData::parsing_rule() noexcept {
  using boost::spirit::as_string;
  using boost::spirit::ascii::space;
  using boost::spirit::qi::char_;
  using boost::spirit::qi::lexeme;

  static ParseRule<std::string> const name_rule = as_string[+(char_ - space - ',')];
  static ParseRule<std::string> const val_rule  = as_string[+(char_ - '}')];

  static ParseRule<TMSVRJsonData> rule = "{\"Item\":" >> name_rule >> ",\"Value\":" >> val_rule >> '}';
  return rule;
}

}  // namespace tmr_listener

namespace tmr_listener {
namespace detail {

ParseRule<TMSVRTag::DataFormat>& TMSVRTag::DataFormat::parsing_rule() noexcept {
  using boost::phoenix::static_cast_;
  using boost::spirit::qi::_val;
  using boost::spirit::qi::char_;
  using boost::spirit::qi::int_;
  using boost::spirit::qi::lit;

  using JsonArray = std::vector<TMSVRJsonData>;

  static ParseRule<std::string> const id_rule = +(char_ - ",");
  static ParseRule<Mode> const mode_rule      = int_[_val = static_cast_<Mode>(boost::spirit::qi::_1)];

  static ParseRule<std::string> const raw_content_rule = +(char_ - ",*");

  static ParseRule<DataFormat> rule = id_rule >> ',' >> mode_rule >> ',' >> raw_content_rule;
  return rule;
}

TMSVRTag::DataFormat::Data TMSVRTag::DataFormat::parse_raw_content(std::string t_raw_content) noexcept {
  using namespace boost::spirit::qi;
  using boost::spirit::ascii::space;

  using JsonArray = std::vector<TMSVRJsonData>;

  static ParseRule<JsonArray> const data_rule = '[' >> *(TMSVRJsonData::parsing_rule() >> *lit(',')) >> ']';

  Data ret_val;
  bool const full_match = phrase_parse(t_raw_content.begin(), t_raw_content.end(), data_rule, space, ret_val);

  return ret_val;
}

void TMSVRTag::BuildRule::check(std::vector<std::string>& t_content_holder, TMSVRJsonData const& t_input) noexcept {
  if (t_content_holder.size() == 1) {
    t_content_holder.push_back("3");
  }

  t_content_holder.push_back(t_input.to_str());
}

void TMSVRTag::BuildRule::check(std::vector<std::string>& t_content_holder, TMSVRJsonReadReq const& t_input) noexcept {
  if (t_content_holder.size() == 1) {
    t_content_holder.push_back("13");
  }

  t_content_holder.push_back(t_input.to_str());
}

std::string TMSVRTag::assemble(std::vector<std::string> const& t_content) noexcept {
  auto const id      = t_content[0];
  auto const mode    = t_content[1];
  auto const content = std::vector<std::string>{t_content.begin() + 2, t_content.end()};
  return id + ',' + mode + ",[" + boost::algorithm::join(content, ",") + ']';
}

}  // namespace detail
}  // namespace tmr_listener

BOOST_FUSION_ADAPT_STRUCT(tmr_listener::TMSVRJsonData, item_, value_)
BOOST_FUSION_ADAPT_STRUCT(tmr_listener::detail::TMSVRTag::DataFormat, id_, mode_, raw_content_)