#ifndef tmr_listener_HPP_
#define tmr_listener_HPP_

#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/thread.hpp>
#include <chrono>
#include <memory>

#include "tmr_listener/tmr_listener_handle.hpp"
#include "tmr_tcp_comm/tmr_tcp_comm.hpp"

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

namespace tmr_listener {

class TMRobotListener {
 private:
  class ScriptExitHandler final : public ListenerHandle {
   protected:
    MessagePtr generate_cmd(MessageStatus /*unused*/) override {
      using namespace motion_function;
      return TMSCT << ID{"TMRobotListener_DefaultHandler"} << ScriptExit();
    }

    Decision start_task(std::vector<std::string> const & /*unused*/) override { return Decision::Ignore; }
  };

  using TMTaskHandler        = boost::shared_ptr<ListenerHandle>;
  using TMTaskHandlerArray_t = std::vector<TMTaskHandler>;

  static constexpr auto TMR_INIT_MSG_ID  = "0";    /* !< TM robot message id when first enter listen node */
  static constexpr auto MESSAGE_END_BYTE = "\r\n"; /* !< TM script message ends with this 2 bytes, \r\n */

  static constexpr auto HEADER_INDEX       = 0; /*!< Index of the HEADER byte in the tokenized rx msg */
  static constexpr auto LENGTH_INDEX       = 1; /*!< Index of the LENGTH byte in the tokenized rx msg */
  static constexpr auto DATA_START_INDEX   = 2; /*!< Index of the Data byte in tokenized rx msg */
  static constexpr auto ID_INDEX           = DATA_START_INDEX + 0;
  static constexpr auto SCRIPT_START_INDEX = DATA_START_INDEX + 1;

  TMRobotTCP tcp_comm_; /*!< TM TCP communication object */

  /**
   * @brief This function parses the message sent by TM robot, once entered listen node, it will initiate
   *        the write process, otherwise it continues listening to incomming packet
   *
   * @param t_input message sent by TM robot
   */
  void parse_msg(std::string const &t_input) noexcept;

  /**
   * @brief This function gets called once the message to write to TM robot finished transfer
   *
   * @param t_byte_writtened bytes written to TM robot
   */
  void finished_transfer_callback(size_t t_byte_writtened) noexcept;

  /**
   * @brief Get all the plugin object
   */
  TMTaskHandlerArray_t get_all_plugins();

  ros::NodeHandle private_nh_{"~/"};
  pluginlib::ClassLoader<ListenerHandle> class_loader_{"tmr_listener", "tmr_listener::ListenerHandle"};

  TMTaskHandler default_task_handler_{boost::make_shared<ScriptExitHandler>()};
  TMTaskHandlerArray_t task_handlers_{};
  TMTaskHandler current_task_handler_{};

 public:
  static constexpr auto DEFAULT_IP_ADDRESS = "192.168.1.2";
  static constexpr auto LISTENER_PORT      = 5890;

  explicit TMRobotListener(std::string const &t_ip_addr = DEFAULT_IP_ADDRESS) noexcept
    : tcp_comm_{TMRobotTCP::Callback{boost::bind(&TMRobotListener::parse_msg, this, _1),
                                     boost::bind(&TMRobotListener::finished_transfer_callback, this, _1)},
                LISTENER_PORT, t_ip_addr},
      task_handlers_{get_all_plugins()} {}

  /**
   * @brief This function is the entry point to the TCP/IP connection, it initiates the thread loop and runs io services
   *        in the background
   */
  void start();
};

}  // namespace tmr_listener

#endif