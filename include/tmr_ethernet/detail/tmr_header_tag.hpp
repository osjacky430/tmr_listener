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

  struct DataFormat {
    using DataTableFormat = TMSVRJsonData;  // @todo: extend to string, and possibly, binary mode
    using Data            = std::vector<DataTableFormat>;

    std::string id_;
    int mode_;
    std::string raw_content_;

    static ParseRule<DataFormat>& parsing_rule() noexcept;

    static Data parse_raw_content(std::string t_input) noexcept;
  };

  static auto create_builder(ID const& t_id) noexcept {
    prototype::MessageBuilder<TMSVRTag> ret_val{};
    ret_val.result_.content_.push_back(t_id.id_);
    return ret_val;
  }

  // @todo refactor
  template <typename T>
  static constexpr bool check_acceptance() noexcept {
    static_assert(std::is_same<T, TMSVRJsonData>::value, "Only support JSON format for TMSVR currently");
    return true;
  }

  static std::string assemble(std::vector<std::string> const& t_content) noexcept;
};

}  // namespace detail
}  // namespace tmr_listener

#endif