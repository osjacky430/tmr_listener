#ifndef TMR_ETH_RW_HPP_
#define TMR_ETH_RW_HPP_

#include "tmr_ethernet/detail/tmr_fwd.hpp"
#include "tmr_ethernet/detail/tmr_header_tag.hpp"
#include "tmr_ext_script/detail/tmr_function.hpp"

#include <string>
#include <utility>

namespace tmr_listener {

enum class Mode {
  ServerResponse = 00, /*!< Indicates the server responds to the client comand in string format */
  Binary         = 01, /*!< Indicates the content data type in binary format */
  String         = 02, /*!< Indicates the content data type in string format */
  Json           = 03, /*!< Indicates the content data type in JSON format */
  ReadBinary     = 11, /*!< Indicates the content data type in binary format (Request read) */
  ReadString     = 12, /*!< Indicates the content data type in string format (Request read) */
  ReadJson       = 13  /*!< Indicates the content data type in JSON format (Request read) */
};

/**
 * @brief This class holds key-value pair for TM ethernet data, it is used in TMSVR message generation and TMSVR data
 *        table storagee class
 *
 * @note  For data table storage class, we can default construct, then assign.
 *        For message generation, we need to check if the input param is quoted or not.
 */
struct TMSVRJsonData {
  std::string item_;
  std::string value_;

  static ParseRule<TMSVRJsonData>& parsing_rule() noexcept;

  std::string to_str() const noexcept {
    using namespace std::string_literals;
    return R"({"Item":)"s + this->item_ + R"(,"Value":)"s + this->value_ + "}"s;
  }
};

using TMSVRJsonWriteReq = TMSVRJsonData;

/**
 * @brief This function creates a TMSVRJsonWriteReq, it is intended to be used for message generation since it will
 *        check if the input string is quoted or not
 *
 * @param t_item  key
 * @param t_value value
 * @return TMSVRJsonWriteReq object
 */
static auto generate_write_req(std::string const& t_item, std::string const& t_value) {
  if (t_item.empty() or t_value.empty()) {
    throw std::invalid_argument("empty item and empty value is not allowed");
  }

  auto const is_string_quoted = [](std::string const& t_input) {
    return t_input.find_first_of('\"') == 0 and t_input.find_last_of('\"') == t_input.size() - 1;
  };

  return TMSVRJsonWriteReq{is_string_quoted(t_item) ? t_item : '\"' + t_item + '\"',
                           is_string_quoted(t_value) ? t_value : '\"' + t_value + '\"'};
}

/**
 * @brief This class holds the item entry for read request
 */
struct TMSVRJsonReadReq {
  std::string item_;

  std::string to_str() const noexcept {
    using namespace std::string_literals;
    return R"({"Item":)"s + this->item_ + "}"s;
  }
};

/**
 * @brief This function create TMSVRJsonReadReq object, it is intended to be used for message generation since it will
 *        check if the input string is quoted or not
 *
 * @param t_item  variable name to request value
 * @return TMSVRJsonReadReq
 */
static auto generate_read_req(std::string const& t_item) {
  if (t_item.empty()) {
    throw std::invalid_argument("empty item and empty value is not allowed");
  }

  auto const is_string_quoted = [](std::string const& t_input) {
    return t_input.find_first_of('\"') == 0 and t_input.find_last_of('\"') == t_input.size() - 1;
  };

  return TMSVRJsonReadReq{is_string_quoted(t_item) ? t_item : '\"' + t_item + '\"'};
}

}  // namespace tmr_listener

#endif