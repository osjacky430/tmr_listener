#ifndef TMR_CMD_LINE_OPTION_HPP_
#define TMR_CMD_LINE_OPTION_HPP_

#include <boost/program_options.hpp>
#include <string>

#include "tmr_tcp_comm/tmr_tcp_comm.hpp"

namespace tmr_listener {

inline auto common_option() noexcept {
  using namespace boost::program_options;
  options_description common{"Common options"};
  common.add_options()                           //
    ("help", "Show this help message and exit")  //
    ("ip", value<std::string>()->default_value(TMRobotTCP::DEFAULT_IP_ADDRESS),
     "IP address of the TM robot")       //
    ("verbose", "Print debug message");  //

  return common;
}

}  // namespace tmr_listener

#endif