#ifndef TMR_TCP_COMM_HPP_
#define TMR_TCP_COMM_HPP_

#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>

#include <chrono>
#include <string>

namespace tmr_listener {

class TMRobotTCP {
 public:
  struct Callback {
    boost::function<void(std::string const &)> msg_recved_;
    boost::function<void(size_t const)> msg_sent_ = {};
  };

 private:
  boost::asio::io_service io_service_;
  boost::asio::ip::address robot_address_;
  boost::asio::ip::tcp::endpoint tm_robot_;
  boost::asio::ip::tcp::socket listener_{io_service_};
  boost::asio::steady_timer ros_heartbeat_timer_{io_service_};
  boost::asio::streambuf input_buffer_;
  std::string output_buffer_;

  boost::thread comm_thread_;

  Callback cb_;

  static constexpr auto MESSAGE_END_BYTE = "\r\n"; /* !< TM script message ends with this 2 bytes, \r\n */

  /**
   * @brief This function extract buffer data from boost asio streambuf and store the result to std::string
   *
   * @param t_buffer  Buffer to extract data
   * @param t_byte_to_extract Number of byte to extract
   * @return extracted data
   */
  static std::string extract_buffer_data(boost::asio::streambuf &t_buffer, size_t t_byte_to_extract) noexcept;

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
   * @brief This function handles reconnection when fail situation detected during read/write stage
   */
  void reconnect() noexcept;

  /**
   * @brief This function stops the timer and closes the socket
   */
  void stop() noexcept;

  static constexpr auto HEARTBEAT_INTERVAL() { return std::chrono::milliseconds(100); }

 public:
  static constexpr auto DEFAULT_IP_ADDRESS = "192.168.1.2";

  explicit TMRobotTCP(Callback t_cb, unsigned short const t_port,
                      std::string const &t_ip_addr = DEFAULT_IP_ADDRESS) noexcept
    : robot_address_{boost::asio::ip::address::from_string(t_ip_addr)},
      tm_robot_{robot_address_, t_port},
      cb_{std::move(t_cb)} {}

  /**
   * @brief Thread function that initializes and runs the IO services
   */
  void start_tcp_comm();

  void write(std::string const &t_value) noexcept {
    using namespace boost::asio::placeholders;

    this->output_buffer_ = t_value;
    boost::asio::async_write(this->listener_, boost::asio::buffer(this->output_buffer_),
                             boost::bind(&TMRobotTCP::handle_write, this, error, bytes_transferred));
  }
};

}  // namespace tmr_listener

#endif