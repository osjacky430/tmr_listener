#ifndef TMR_TCP_COMM_HPP_
#define TMR_TCP_COMM_HPP_

#include <boost/asio.hpp>
#include <boost/asio/high_resolution_timer.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

namespace tmr_listener {

class TMRobotTCP {
 public:
  struct Callback {
    boost::function<void(std::string const &)> msg_recved_;
    boost::function<void(size_t const)> msg_sent_ = {};
    boost::function<void()> disconnected_         = {};
  };

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

  /**
   * @brief This function closes the socket
   */
  void stop();

  bool is_connected() const noexcept { return this->is_connected_.load(); }

  /**
   * @brief This function write the input value to TM server
   *
   * @param t_value string to write to server
   *
   * @note  A buffer object does not have any ownership of the memory it refers to. It is the responsibility of the
   *        application to ensure the memory region remains valid until it is no longer required for an I/O operation.
   *        When the memory is no longer available, the buffer is said to have been invalidated. Therefore, we need to
   *        move t_value into output buffer.
   */
  void write(std::string t_value) {
    this->output_buffer_ = std::move(t_value);
    boost::asio::async_write(
      this->listener_, boost::asio::buffer(this->output_buffer_),
      [this](auto t_err, auto t_byte_transferred) { this->handle_write(t_err, t_byte_transferred); });
  }

 private:
  boost::asio::io_service io_service_;
  boost::asio::ip::address robot_address_;
  boost::asio::ip::tcp::endpoint tm_robot_;
  boost::asio::ip::tcp::socket listener_{io_service_};
  boost::asio::high_resolution_timer conn_timeout_timer_{io_service_};
  boost::asio::streambuf input_buffer_;
  std::string output_buffer_;

  Callback cb_;
  std::atomic_bool is_connected_{false};

  static constexpr auto MESSAGE_END_BYTE = "\r\n"; /* !< TM script message ends with this 2 bytes, \r\n */
  static const std::chrono::milliseconds CONNECTION_TIMEOUT;

  /**
   * @brief This function extract buffer data from boost asio streambuf and store the result to std::string
   *
   * @param t_buffer  Buffer to extract data
   * @param t_byte_to_extract Number of byte to extract
   * @return extracted data
   */
  static std::string extract_buffer_data(boost::asio::streambuf &t_buffer, size_t t_byte_to_extract) noexcept;

  /**
   * @brief This function is called when connection is not success within the time CONNECTION_TIMEOUT
   *
   * @param t_err system error happened during connection, not important here, the error is silenced
   */
  void handle_connection_timeout(boost::system::error_code const &t_err);

  /**
   * @brief This function handles the connection and initiate the read process if the connection succeeded
   *
   * @param t_err system error happened during connection
   */
  void handle_connection(boost::system::error_code const &t_err);

  /**
   * @brief This function handles the read process of TCP connection, once entered listen node, it will initiate the
   *        write process, otherwise it continues listening to incomming packet
   *
   * @param t_err system error happened during read process
   * @param t_byte_transfered Number of byte read from TM robot
   */
  void handle_read(boost::system::error_code const &t_err, size_t t_byte_transfered);

  /**
   * @brief This function handles the write process of TCP connection, it will continue writing once triggered, until
   *        current_handler_ is reset.
   *
   * @param t_err system error happened during write process
   * @param t_byte_writtened Number of byte written to TM robot
   */
  void handle_write(boost::system::error_code const &t_err, size_t t_byte_writtened);

  /**
   * @brief This function handles reconnection when fail situation detected during read/write stage
   */
  void new_connection();

  std::string print_ip_port() const noexcept {
    return this->tm_robot_.address().to_string() + ':' + std::to_string(this->tm_robot_.port());
  }
};

}  // namespace tmr_listener

#endif