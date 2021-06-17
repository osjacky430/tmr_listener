#ifndef TMR_ETH_SLAVE_HPP_
#define TMR_ETH_SLAVE_HPP_

#include <string>

#include <boost/thread.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "tmr_ethernet/detail/tmr_header_tag.hpp"
#include "tmr_ethernet/tmr_eth_rw.hpp"
#include "tmr_listener/EthernetSlaveCmd.h"
#include "tmr_listener/JsonDataArray.h"
#include "tmr_prototype/tmr_header.hpp"
#include "tmr_tcp_comm/tmr_tcp_comm.hpp"

namespace tmr_listener {

class TMRobotEthSlave {
  static constexpr unsigned short ETHERNET_SLAVE_PORT = 5891;

  TMRobotTCP comm_;

  ros::NodeHandle private_nh_{"~"};
  ros::Publisher raw_data_table_pub_{private_nh_.advertise<std_msgs::String>("raw_data_table", 1)};
  ros::Publisher processed_data_table_pub_{private_nh_.advertise<tmr_listener::JsonDataArray>("parsed_data_table", 1)};
  ros::ServiceServer tmsvr_cmd_srv_{private_nh_.advertiseService("tmsvr_cmd", &TMRobotEthSlave::send_tmsvr_cmd, this)};

  bool responded_ = false;
  typename TMSVRPacket::DataFrame server_response_;
  boost::condition_variable response_signal_;
  boost::mutex rx_buffer_mutex_;
  boost::asio::signal_set sigterm_handler_{comm_.get_io_service(), SIGINT};

  void parse_input_msg(std::string const& t_input) noexcept;

  /**
   * @brief TMSVR command service
   *
   * @param t_req   Contains all information to generate TMSVR command
   * @param t_resp  Empty for request read. As for write request, this contains the response from server
   * @return true   Service call succeeded
   * @return false  Service call failed, possibly due to invalid context, e.g. empty string
   */
  bool send_tmsvr_cmd(EthernetSlaveCmdRequest& t_req, EthernetSlaveCmdResponse& t_resp);

 public:
  explicit TMRobotEthSlave(std::string const& t_ip) noexcept
    : comm_{TMRobotTCP::Callback{boost::bind(&TMRobotEthSlave::parse_input_msg, this, _1)}, ETHERNET_SLAVE_PORT, t_ip} {
  }

  /**
   * @brief This function is the entry point to the TCP/IP connection, it initiates the thread loop and runs io services
   *        in the background
   */
  void start() noexcept;

  /**
   * @brief Since the service might be waiting for TM server response forever, we need to take care of SIGINT manually
   *        these two things happened at the same time
   *
   * @param t_err Error code
   */
  void sigterm_handler(boost::system::error_code t_err, int /* unused */) noexcept;
};

}  // namespace tmr_listener

BOOST_FUSION_ADAPT_STRUCT(tmr_listener::TMSVRPacket, length_, data_, checksum_)

#endif
