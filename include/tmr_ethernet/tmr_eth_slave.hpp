#ifndef TMR_ETH_SLAVE_HPP_
#define TMR_ETH_SLAVE_HPP_

#include <string>

#include <boost/thread.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "tmr_ethernet/detail/tmr_header_tag.hpp"
#include "tmr_ethernet/tmr_eth_rw.hpp"
#include "tmr_listener/JsonDataArray.h"
#include "tmr_prototype/tmr_header.hpp"
#include "tmr_tcp_comm/tmr_tcp_comm.hpp"

namespace tmr_listener {

using TMSVRPacket = prototype::Header<detail::TMSVRTag>::Packet;

static constexpr auto TMSVR = prototype::Header<detail::TMSVRTag>{};

}  // namespace tmr_listener

namespace tmr_listener {

class TMRobotEthSlave {
  static constexpr unsigned short ETHERNET_SLAVE_PORT = 5891;

  TMRobotTCP comm_;

  ros::NodeHandle private_nh_{"~"};
  ros::Publisher raw_data_table_pub_{private_nh_.advertise<std_msgs::String>("raw_data_table", 1)};
  ros::Publisher processed_data_table_pub_{private_nh_.advertise<tmr_listener::JsonDataArray>("parsed_data_table", 1)};
  void parse_input_msg(std::string const& t_input) noexcept;

 public:
  explicit TMRobotEthSlave(std::string const& t_ip) noexcept
    : comm_{TMRobotTCP::Callback{boost::bind(&TMRobotEthSlave::parse_input_msg, this, _1)}, ETHERNET_SLAVE_PORT, t_ip} {
  }

  void start() noexcept;
};

}  // namespace tmr_listener

BOOST_FUSION_ADAPT_STRUCT(tmr_listener::TMSVRPacket, length_, data_, checksum_)

#endif
