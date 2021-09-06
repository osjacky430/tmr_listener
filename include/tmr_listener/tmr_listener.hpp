#ifndef TMR_LISTENER_HPP_
#define TMR_LISTENER_HPP_

#include <boost/shared_ptr.hpp>
#include <type_traits>

#include "tmr_listener/SubCmdDataMsg.h"
#include "tmr_listener/tmr_listener_handle.hpp"
#include "tmr_tcp_comm/tmr_tcp_comm.hpp"
#include "tmr_utility/tmr_parser.hpp"

#include <pluginlib/class_loader.h>
#include <ros/node_handle.h>

namespace tmr_listener {

using TMTaskHandler        = boost::shared_ptr<ListenerHandle>;
using TMTaskHandlerArray_t = std::vector<TMTaskHandler>;

/**
 * @brief Interface of TMR plugin manager, inherit from this to make your custom plugin manager
 */
struct TMRPluginManagerBase {
  /**
   * @brief Get all plugins
   *
   * @return TMTaskHandlerArray_t
   */
  virtual TMTaskHandlerArray_t get_all_plugins() const = 0;

  /**
   * @brief   This function finds the listner handle that is willing to take on task according to input string
   *
   * @param  t_input  Enter node message sent from TM robot listen node
   * @return TMTaskHandler that accepted the test
   *
   * @note   This function doesn't catch any exception. Implementer should handle exception within the function.
   *         Implementer should return their own default handler if non is matched, returning empty shared_ptr will
   *         result in exception thrown by TMRListener after calling this function.
   */
  virtual TMTaskHandler find_task_handler(std::string const &t_input) const = 0;

  TMRPluginManagerBase()                                        = default;
  TMRPluginManagerBase(TMRPluginManagerBase const & /*unused*/) = default;
  TMRPluginManagerBase(TMRPluginManagerBase && /*unused*/)      = default;

  TMRPluginManagerBase &operator=(TMRPluginManagerBase const & /*unused*/) = default;
  TMRPluginManagerBase &operator=(TMRPluginManagerBase && /*unused*/) = default;

  virtual ~TMRPluginManagerBase() = default;
};

/**
 * @brief Default handler used by RTLibPluginManager
 */
class ScriptExitHandler final : public ListenerHandle {
 protected:
  MessagePtr generate_cmd(MessageStatus /*unused*/) noexcept override {
    using namespace motion_function;
    return TMSCT << ID{"TMRobotListener_DefaultHandler"} << ScriptExit();
  }

  Decision start_task(std::string const & /*unused*/) noexcept override { return Decision::Ignore; }
};

/**
 * @brief Default plugin manager used by TMRobotListener, it loads plugin via pluginlib. If no loaded plugin accept
 *        incoming response when enter listen node, it will return default handler, which generates ScriptExit().
 */
class RTLibPluginManager final : public TMRPluginManagerBase {
  TMTaskHandlerArray_t load_plugins();

  pluginlib::ClassLoader<ListenerHandle> class_loader_{"tmr_listener", "tmr_listener::ListenerHandle"};
  TMTaskHandlerArray_t all_plugins_{load_plugins()};
  TMTaskHandler default_handler_{boost::make_shared<ScriptExitHandler>()};

 public:
  TMTaskHandlerArray_t get_all_plugins() const noexcept override { return this->all_plugins_; }
  TMTaskHandler find_task_handler(std::string const &t_input) const noexcept override;
};

using TMRPluginManagerBasePtr = boost::shared_ptr<TMRPluginManagerBase>;

/**
 * @brief This class integrates (1) message parsing, (2) communication and (3) plugin management together.
 *
 * @details Default behaviour of plugin management uses pluginlib for plugin management to separate library code from
 *          application code, you can provide your own plugin management by inheriting TMRPluginManagerBase if pluginlib
 *          is an overkill, e.g. you only need one plugin.
 *
 *          Otherwise, just include `tmr_listener.launch`, or put `<node pkg="tmr_listener" type="tmr_listener_node"
 *          name="tmr_listener" args="$(arg ip_arg)"/>` in your launch file and you're good to go.
 */
class TMRobotListener {
 public:
  using ListenData          = boost::variant<TMSCTPacket, TMSTAPacket, CPERRPacket>;
  using TMRListenDataParser = ParseRule<ListenData>;

  /**
   * @brief This function parses input string into one of the packets, i.e., TMSCT, TMSTA, or CPERR
   *
   * @param t_input string to parse
   * @return ListenData
   *
   * @todo throw if not fully matched in the future
   */
  static ListenData parse(std::string t_input);

  /**
   * @brief This function returns the parsing rule of listen message
   *
   * @return TMRListenDataParser
   */
  static TMRListenDataParser parsing_rule() noexcept;

  static constexpr auto LISTENER_PORT = 5890;

  /**
   * @brief Construct a new TMRobotListener object
   *
   * @param t_ip_addr The ip address of the TM robot
   * @param t_plugin_manager  The ListenerHandle plugin manager, default uses plugin lib to load plugins
   */
  explicit TMRobotListener(
    std::string t_ip_addr, ros::NodeHandle t_nh = ros::NodeHandle{"~"},
    TMRPluginManagerBasePtr t_plugin_manager = boost::make_shared<RTLibPluginManager>()) noexcept;

  /**
   * @brief This function is the entry point to the TCP/IP connection, it initiates the thread loop and runs io
   * services in the background
   */
  void start();

  /**
   * @note  explicitly defaulted to deal with incomplete type and unqiue_ptr
   */
  ~TMRobotListener();

 private:
  static constexpr auto TMR_INIT_MSG_ID = "0"; /* !< TM robot message id when first enter listen node */

  /**
   * @brief This class make suitable responses for each packet type. It is basically a static_visitor that operates on
   *        ListenData
   */
  class PacketVisitor;
  friend PacketVisitor;

  TMRobotTCP tcp_comm_;                    /*!< TM TCP communication object */
  TMRPluginManagerBasePtr plugin_manager_; /*!< TM plugin manager holder */
  TMTaskHandler current_task_handler_{};   /*!< plugin that is currently handling incoming packet */
  std::unique_ptr<PacketVisitor> visitor_; /*!< Packet visitor operator */

  /**
   * @brief This function parses the message sent by TM robot, once entered listen node, it will initiate
   *        the write process, otherwise it continues listening to incomming packet
   *
   * @param t_input message sent by TM robot
   *
   * @note  This noexcept operator is only to remind me to determine whether parse() should throw or ignore not fully
   *        matched case
   */
  void receive_tm_msg_callback(std::string const &t_input) noexcept(noexcept(parse(t_input)));

  /**
   * @brief This function gets called once the message to write to TM robot finished transfer
   *
   * @param t_byte_writtened bytes written to TM robot
   */
  void finished_transfer_callback(size_t t_byte_writtened) noexcept;

  /**
   * @brief This function gets called if disconnect event happened, see ListenerHandle::handle_disconnect
   */
  void disconnected_callback() noexcept;
};

}  // namespace tmr_listener

#endif