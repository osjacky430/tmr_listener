#ifndef TMR_ETH_RW_HPP_
#define TMR_ETH_RW_HPP_

#include "tmr_ethernet/detail/tmr_fwd.hpp"
#include "tmr_ethernet/detail/tmr_header_tag.hpp"
#include "tmr_ext_script/detail/tmr_function.hpp"

#include <string>

namespace tmr_listener {

struct TMSVRJsonData {
  std::string item_;
  std::string value_;

  static ParseRule<TMSVRJsonData>& parsing_rule() noexcept;

  std::string to_str() const noexcept {
    auto const item_entry  = "\"Item\":" + this->item_;
    auto const value_entry = this->value_.empty() ? this->value_ : ",\"Value\":" + this->value_;
    return '{' + item_entry + value_entry + '}';
  }
};

}  // namespace tmr_listener

#endif