#include <ros/ros.h>

#include "tmr_tcp_comm/tmr_tcp_comm.hpp"

namespace tmr_listener {

std::chrono::milliseconds const TMRobotTCP::CONNECTION_TIMEOUT = std::chrono::milliseconds(100);

std::string TMRobotTCP::extract_buffer_data(boost::asio::streambuf &t_buffer, size_t const t_byte_to_extract) noexcept {
  if (t_byte_to_extract > 0) {
    auto const buffer_begin = boost::asio::buffers_begin(t_buffer.data());
    std::string result{buffer_begin, buffer_begin + static_cast<std::ptrdiff_t>(t_byte_to_extract)};
    t_buffer.consume(t_byte_to_extract);

    return result;
  }

  return std::string{""};
}

/**
 * @details Since TM robot starts TCP server when the project starts. If the program sends SYN before server starts, and
 *          starts to wait for SYN-ACK, the server will do nothing because it doesn't receive any packet after it starts
 *          , hence the slow connection establishment. Reconnection takes place when default timeout is reached, which
 *          is unacceptable most of the time.
 */
void TMRobotTCP::handle_connection_timeout(boost::system::error_code const &t_err) {
  if (t_err or not ros::ok() or this->is_connected_) {  // NOLINT boost pre c++11 safe bool idiom
    return;
  }

  ROS_ERROR_STREAM_THROTTLE_NAMED(300.0, "tm_socket_connection",
                                  this->print_ip_port() << " Connection error, reason: Timeout, retrying...");
  this->new_connection();
}

/**
 * @details  After the connection is established, obtain first message when entering listener node, this message serves
 *           as a signal to tell which handler should handle the work
 */
void TMRobotTCP::handle_connection(boost::system::error_code const &t_err) {
  if (not ros::ok() or t_err) {  // NOLINT boost pre c++11 safe bool idiom
    return;
  }

  ROS_INFO_STREAM_NAMED("tm_socket_connection", this->print_ip_port()
                                                  << " Connection success, waiting for server response");

  this->is_connected_.store(true);
  auto const callback = [this](auto t_error, auto t_tx_byte_num) { this->handle_read(t_error, t_tx_byte_num); };
  boost::asio::async_read_until(this->listener_, this->input_buffer_, MESSAGE_END_BYTE, callback);
}

/**
 * @details Since TM robot sends TMSCT message only during listen node, we can assume that if there is no task handler
 *          currently, the incoming TMSCT message can only be the first message sent when entering listen node. With
 *          that said, one thing we need to do is to make sure if we exit script, e.g., by sending ScriptExit(),
 *          current_task_handler_ is reset.
 *
 *          If there is no handler that is willing to handle the current listen node, then default_task_handler_ will
 *          generate the response, sending ScriptExit() immediately to TM robot.
 *
 * @note    The buffer passed to async_read_until is already committed
 * @note    TM robot will send OK message even after ScriptExit()
 */
void TMRobotTCP::handle_read(boost::system::error_code const &t_err, size_t const t_byte_transfered) {
  if (not ros::ok()) {
    return;
  }

  if (not t_err) {  // NOLINT, boost pre c++11 safe bool idiom
    auto const result = TMRobotTCP::extract_buffer_data(this->input_buffer_, t_byte_transfered);
    this->cb_.msg_recved_(result);  // call callback to owner to handle incoming buffer

    // initiate another read process
    auto const callback = [this](auto t_error, auto t_tx_byte_num) { this->handle_read(t_error, t_tx_byte_num); };
    boost::asio::async_read_until(this->listener_, this->input_buffer_, MESSAGE_END_BYTE, callback);
  } else {
    ROS_ERROR_STREAM_NAMED("tm_listener_node", this->print_ip_port() << " Read Error: " << t_err.message());
    ROS_ERROR_STREAM_NAMED("tm_socket_connection", this->print_ip_port() << " Read Error detected, reconnecting...");

    this->new_connection();
  }
}

void TMRobotTCP::handle_write(boost::system::error_code const &t_err, size_t const t_byte_writtened) {
  if (not ros::ok()) {
    return;
  }

  if (not t_err) {              // NOLINT, boost pre c++11 safe bool idiom
    if (this->cb_.msg_sent_) {  // NOLINT, safe bool idiom
      this->cb_.msg_sent_(t_byte_writtened);
    }
  } else {
    ROS_ERROR_STREAM_NAMED("tm_listener_node", this->print_ip_port() << " Write Error: " << t_err.message());
    ROS_ERROR_STREAM_NAMED("tm_socket_connection", this->print_ip_port() << " Write Error detected, reconnecting...");

    this->new_connection();
  }
}

void TMRobotTCP::new_connection() {
  this->stop();
  this->io_service_.dispatch([this]() {
    this->listener_.async_connect(this->tm_robot_, [this](auto t_err) { this->handle_connection(t_err); });
    this->conn_timeout_timer_.expires_from_now(CONNECTION_TIMEOUT);
    this->conn_timeout_timer_.async_wait([this](auto t_err) { this->handle_connection_timeout(t_err); });
  });
}

void TMRobotTCP::stop() {
  this->io_service_.dispatch([this]() {
    if (this->is_connected_) {        // NOLINT, pre c++11 safe bool idiom
      if (this->cb_.disconnected_) {  // NOLINT, pre c++11 safe bool idiom
        this->cb_.disconnected_();
      }

      this->is_connected_.store(false);
    }

    boost::system::error_code ignored;
    this->listener_.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ignored);
    this->listener_.close(ignored);
  });
}

void TMRobotTCP::start_tcp_comm() {
  this->new_connection();

  try {
    this->io_service_.run();
  } catch (std::exception &e) {
    ROS_ERROR_STREAM_COND_NAMED(ros::ok(), "tm_listener_connection",
                                this->print_ip_port() << " Exception: " << e.what());
  }
}

}  // namespace tmr_listener