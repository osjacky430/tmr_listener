#ifndef TMR_ETHERNET_HEADER_TAG_HPP_
#define TMR_ETHERNET_HEADER_TAG_HPP_

#include <boost/fusion/include/adapt_struct.hpp>

#include <string>

#include "tmr_ethernet/detail/tmr_fwd.hpp"
#include "tmr_prototype/tmr_header.hpp"
#include "tmr_utility/tmr_parser.hpp"

namespace tmr_listener {
namespace detail {

struct TMSVRTag {
  static constexpr auto NAME() { return "$TMSVR"; }

  struct DataFrame {
    using DataTableFormat = TMSVRJsonData;  // @todo: extend to string, and possibly, binary mode
    using Data            = std::vector<DataTableFormat>;

    std::string id_;
    Mode mode_{};
    std::string raw_content_;

    static ParseRule<DataFrame>& parsing_rule() noexcept;

    static Data parse_raw_content(std::string t_raw_content);
  };

  struct BuildRule {
    static void check(std::vector<std::string>& t_content_holder, TMSVRJsonData const& t_input) noexcept;

    static void check(std::vector<std::string>& t_content_holder, TMSVRJsonReadReq const& t_input) noexcept;
  };

  static auto create_builder(ID const& t_id) noexcept {
    prototype::MessageBuilder<TMSVRTag> ret_val{};
    ret_val.result_.content_.push_back(t_id.id_);
    return ret_val;
  }

  static std::string assemble(std::vector<std::string> const& t_content) noexcept;
};

}  // namespace detail
}  // namespace tmr_listener

#endif