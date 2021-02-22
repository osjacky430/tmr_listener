#ifndef TM_ROBOT_LISTENER_HPP__
#define TM_ROBOT_LISTENER_HPP__

#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/range/adaptors.hpp>
#include <boost/thread.hpp>
#include <chrono>
#include <memory>

#include "tmr_listener_handle/tmr_listener_handle.hpp"

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

namespace tm_robot_listener {

class TMRobotListener {
 private:
  class ScriptExitHandler final : public ListenerHandle {
   protected:
    motion_function::BaseHeaderProductPtr generate_cmd(MessageStatus /*unused*/) override {
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

  /**
   * @brief This function checks ros::ok periodically to shutdown the connection immediately
   *
   * @param t_err system error happened when invoking timer
   */
  void check_ros_heartbeat(boost::system::error_code const &t_err) noexcept;

  /**
   * @brief This function handles the connection and initiate the read process if the connection succeeded
   *
   * @param t_err system error happened during connection
   */
  void handle_connection(boost::system::error_code const &t_err) noexcept;

  /**
   * @brief This function extract buffer data from boost asio streambuf and store the result to std::string
   *
   * @param t_buffer  Buffer to extract data
   * @param t_byte_to_extract Number of byte to extract
   * @return extracted data
   */
  static std::string extract_buffer_data(boost::asio::streambuf &t_buffer, size_t t_byte_to_extract) noexcept;

  /**
   * @brief This function handles the read process of TCP connection, once entered listen node, it will initiate the
   *        write process, otherwise it continues listening to incomming packet
   *
   * @param t_err system error happened during read process
   * @param t_byte_transfered Number of byte read from TM robot
   */
  void handle_read(boost::system::error_code const &t_err, size_t t_byte_transfered) noexcept;

  /**
   * @brief This function handles the write process of TCP connection, it will continue writing once triggered, until
   *        current_handler_ is reset.
   *
   * @param t_err system error happened during write process
   * @param t_byte_writtened Number of byte written to TM robot
   */
  void handle_write(boost::system::error_code const &t_err, size_t t_byte_writtened) noexcept;

  /**
   * @brief Thread function that initializes and runs the IO services
   */
  void listener_node();

  /**
   * @brief This function handles reconnection when fail situation detected during read/write stage
   */
  void reconnect() noexcept;

  /**
   * @brief Get the all plugin object
   */
  auto get_all_plugins() {
    using boost::adaptors::transformed;
    auto const plugin_transformer = [this](auto const &t_name) { return this->class_loader_.createInstance(t_name); };
    auto const plugin_names       = this->private_nh_.param("listener_handles", std::vector<std::string>{});
    ROS_DEBUG_STREAM_NAMED("tm_robot_listener", "plugin num: " << plugin_names.size());

    auto const plugins = plugin_names | transformed(plugin_transformer);
    return TMTaskHandlerArray_t{plugins.begin(), plugins.end()};
  }

  boost::asio::io_service io_service_;
  boost::asio::ip::address robot_address_;
  boost::asio::ip::tcp::endpoint tm_robot_{robot_address_, LISTENER_PORT};
  boost::asio::ip::tcp::socket listener_{io_service_};
  boost::asio::steady_timer ros_heartbeat_timer_{io_service_};
  boost::asio::streambuf input_buffer_;
  std::string output_buffer_;

  boost::thread listener_node_thread_;

  ros::NodeHandle private_nh_{"~/"};
  pluginlib::ClassLoader<ListenerHandle> class_loader_{"tm_robot_listener", "tm_robot_listener::ListenerHandle"};

  TMTaskHandler default_task_handler_{boost::make_shared<ScriptExitHandler>()};
  TMTaskHandlerArray_t task_handlers_{};
  TMTaskHandler current_task_handler_{};

 public:
  static constexpr auto HEARTBEAT_INTERVAL() { return std::chrono::milliseconds(100); }
  static constexpr auto DEFAULT_IP_ADDRESS = "192.168.1.2";
  static constexpr auto LISTENER_PORT      = 5890;

  explicit TMRobotListener(std::string const &t_ip_addr = DEFAULT_IP_ADDRESS) noexcept
    : robot_address_{boost::asio::ip::address::from_string(t_ip_addr)}, task_handlers_{get_all_plugins()} {}

  /**
   * @brief This function is the entry point to the TCP/IP connection, it initiates the thread loop and runs io services
   *        in the background
   */
  void start();

  /**
   * @brief This function stops the timer and closes the socket
   */
  void stop() noexcept;
};

}  // namespace tm_robot_listener

#endif