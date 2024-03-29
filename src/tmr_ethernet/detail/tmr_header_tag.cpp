#include <functional>

#include <boost/algorithm/algorithm.hpp>
#include <boost/phoenix.hpp>

#include "tmr_ethernet/detail/tmr_header_tag.hpp"
#include "tmr_ethernet/tmr_eth_rw.hpp"

namespace tmr_listener {

using boost::phoenix::static_cast_;
using boost::spirit::as_string;
using boost::spirit::ascii::space;
using boost::spirit::qi::_val;
using boost::spirit::qi::char_;
using boost::spirit::qi::int_;
using boost::spirit::qi::lit;

ParseRule<TMSVRJsonData>& TMSVRJsonData::parsing_rule() noexcept {
  static ParseRule<std::string> const name_rule = as_string[+(char_ - space - ',')];
  static ParseRule<std::string> const val_rule  = as_string[+(char_ - '}')];

  static ParseRule<TMSVRJsonData> rule = "{\"Item\":" >> name_rule >> ",\"Value\":" >> val_rule >> '}';
  return rule;
}

}  // namespace tmr_listener

namespace tmr_listener {
namespace detail {

/**
 * @details The parsing rule only separate data content from id, mode and checksum, I am trying to delay parsing real
 *          content as much as possilbe. Doing so may (haven't think thoroughly, though) provide more granularity over
 *          the control flow.
 */
ParseRule<TMSVRTag::DataFrame>& TMSVRTag::DataFrame::parsing_rule() noexcept {
  static ParseRule<std::string> const id_rule = +(char_ - ",");
  static ParseRule<Mode> const mode_rule      = int_[_val = static_cast_<Mode>(boost::spirit::qi::_1)];

  static ParseRule<std::string> const raw_content_rule = +(char_ - ",*");

  static ParseRule<DataFrame> rule = id_rule >> ',' >> mode_rule >> ',' >> raw_content_rule;
  return rule;
}

TMSVRTag::DataFrame::Data TMSVRTag::DataFrame::parse_raw_content(std::string t_raw) {
  using JsonArray = std::vector<TMSVRJsonData>;

  static ParseRule<JsonArray> const data_rule = '[' >> *(TMSVRJsonData::parsing_rule() >> *lit(',')) >> ']';

  Data ret_val;
  [[gnu::unused]] bool const full_match = phrase_parse(t_raw.begin(), t_raw.end(), data_rule, space, ret_val);
  assert(full_match);  // full_match will be true like 99.999999% of time, if not, either the parsing rule is bad, or
                       // the input is corrupted, which should be checked before enter this function (@todo implement
                       // the check)

  return ret_val;
}

void TMSVRTag::BuildRule::check(std::vector<std::string>& t_content_holder, TMSVRJsonData const& t_input) noexcept {
  if (t_content_holder.size() == 1) {
    t_content_holder.emplace_back("3");
  }

  t_content_holder.push_back(t_input.to_str());
}

void TMSVRTag::BuildRule::check(std::vector<std::string>& t_content_holder, TMSVRJsonReadReq const& t_input) noexcept {
  if (t_content_holder.size() == 1) {
    t_content_holder.emplace_back("13");
  }

  t_content_holder.push_back(t_input.to_str());
}

std::string TMSVRTag::assemble(std::vector<std::string> const& t_content) noexcept {
  auto const& id     = t_content.at(0);
  auto const& mode   = t_content.at(1);
  auto const content = std::vector<std::string>{t_content.begin() + 2, t_content.end()};
  return id + ',' + mode + ",[" + boost::algorithm::join(content, ",") + ']';
}

}  // namespace detail
}  // namespace tmr_listener

BOOST_FUSION_ADAPT_STRUCT(tmr_listener::TMSVRJsonData, item_, value_)
BOOST_FUSION_ADAPT_STRUCT(tmr_listener::detail::TMSVRTag::DataFrame, id_, mode_, raw_content_)