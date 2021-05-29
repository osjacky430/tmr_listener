#include <boost/range/adaptors.hpp>
#include <ros/ros.h>

#include "tmr_tcp_comm/tmr_tcp_comm.hpp"

namespace tmr_listener {

std::string TMRobotTCP::extract_buffer_data(boost::asio::streambuf &t_buffer, size_t const t_byte_to_extract) noexcept {
  if (t_byte_to_extract > 0) {
    auto const buffer_begin = boost::asio::buffers_begin(t_buffer.data());
    std::string const result{buffer_begin, buffer_begin + t_byte_to_extract};
    t_buffer.consume(t_byte_to_extract);

    return result;
  }

  return std::string{""};
}

void TMRobotTCP::check_ros_heartbeat(boost::system::error_code const &t_err) noexcept {
  using namespace boost::asio::placeholders;

  if (not ros::ok() or t_err) {  // NOLINT boost pre c++11 safe bool idiom
    ROS_ERROR_STREAM_COND_NAMED(ros::ok(), "tm_socket_ros_heartbeat_timer", "Timer Error: " << t_err.message());
    this->stop();
  } else {
    this->ros_heartbeat_timer_.expires_from_now(HEARTBEAT_INTERVAL());
    this->ros_heartbeat_timer_.async_wait(boost::bind(&TMRobotTCP::check_ros_heartbeat, this, error));
  }
}

/**
 * @details This handler always tries to reconnect to the server, unless ROS is stopped.
 *
 *           After the connection is established, obtain first message when entering listener node, this message serves
 *           as a signal to tell which handler should handle the work
 */
void TMRobotTCP::handle_connection(boost::system::error_code const &t_err) noexcept {
  using namespace boost::asio::placeholders;

  if (not ros::ok()) {
    return;
  }

  if (t_err) {  // NOLINT boost pre c++11 safe bool idiom
    ROS_ERROR_STREAM_THROTTLE_NAMED(1.0, "tm_socket_connection",
                                    "Connection error, reason: " << t_err.message() << ", retrying...");

    this->listener_.async_connect(this->tm_robot_, boost::bind(&TMRobotTCP::handle_connection, this, error));
  } else {
    ROS_INFO_STREAM_NAMED("tm_socket_connection", "Connection success, waiting for server response");

    boost::asio::async_read_until(this->listener_, this->input_buffer_, MESSAGE_END_BYTE,
                                  boost::bind(&TMRobotTCP::handle_read, this, error, bytes_transferred));
  }
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
void TMRobotTCP::handle_read(boost::system::error_code const &t_err, size_t const t_byte_transfered) noexcept {
  using namespace boost::asio::placeholders;

  if (not ros::ok()) {
    return;
  }

  if (not t_err) {  // NOLINT, boost pre c++11 safe bool idiom
    auto const result = TMRobotTCP::extract_buffer_data(this->input_buffer_, t_byte_transfered);
    this->cb_.msg_recved_(result);  // call callback to owner to handle incoming buffer

    // initiate another read process
    boost::asio::async_read_until(this->listener_, this->input_buffer_, MESSAGE_END_BYTE,
                                  boost::bind(&TMRobotTCP::handle_read, this, error, bytes_transferred));
  } else {
    ROS_ERROR_STREAM_NAMED("tm_listener_node", "Read Error: " << t_err.message());
    ROS_ERROR_STREAM_NAMED("tm_socket_connection", "Read Error detected, reconnecting...");

    this->reconnect();
  }
}

void TMRobotTCP::handle_write(boost::system::error_code const &t_err, size_t const t_byte_writtened) noexcept {
  if (not ros::ok()) {
    return;
  }

  if (not t_err) {  // NOLINT, boost pre c++11 safe bool idiom
    if (this->cb_.msg_sent_) {
      this->cb_.msg_sent_(t_byte_writtened);
    }
  } else {
    ROS_ERROR_STREAM_NAMED("tm_listener_node", "Write Error: " << t_err.message());
    ROS_ERROR_STREAM_NAMED("tm_socket_connection", "Write Error detected, reconnecting...");

    this->reconnect();
  }
}

void TMRobotTCP::reconnect() noexcept {
  using namespace boost::asio::placeholders;

  boost::system::error_code ignore_error_code;
  this->listener_.close(ignore_error_code);
  this->listener_.async_connect(this->tm_robot_, boost::bind(&TMRobotTCP::handle_connection, this, error));
}

void TMRobotTCP::stop() noexcept {
  boost::system::error_code ignore_error_code;
  this->listener_.close(ignore_error_code);
  this->ros_heartbeat_timer_.cancel();
}

void TMRobotTCP::start_tcp_comm() {
  using namespace boost::asio::placeholders;

  this->listener_.async_connect(this->tm_robot_, boost::bind(&TMRobotTCP::handle_connection, this, error));
  this->ros_heartbeat_timer_.expires_from_now(HEARTBEAT_INTERVAL());
  this->ros_heartbeat_timer_.async_wait(boost::bind(&TMRobotTCP::check_ros_heartbeat, this, error));

  try {
    this->io_service_.run();
  } catch (std::exception &e) {
    ROS_ERROR_STREAM_COND_NAMED(ros::ok(), "tm_listener_connection", "Exception: " << e.what());
  }
}

}  // namespace tmr_listener