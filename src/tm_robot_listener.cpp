#include <boost/asio/buffer.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/tokenizer.hpp>

#include <chrono>
#include <functional>
#include <numeric>

#include "tm_robot_listener/tm_robot_listener.hpp"

namespace {

inline auto get_tokenized_result(std::string const &t_result) noexcept {
  auto const tok_res = boost::tokenizer<boost::char_separator<char>>(t_result, boost::char_separator<char>(","));
  auto const ret_val = std::vector<std::string>{tok_res.begin(), tok_res.end()};
  return ret_val;
}

inline auto strip_crlf(std::string const &t_input) noexcept {
  return t_input.size() < 2 ? t_input : std::string{t_input.begin(), t_input.end() - 2};
}

}  // namespace

namespace tm_robot_listener {

constexpr std::chrono::milliseconds TMRobotListener::HEARTBEAT_INTERVAL;  // before c++17

void TMRobotListener::check_ros_heartbeat(boost::system::error_code const &t_err) noexcept {
  using namespace boost::asio::placeholders;

  if (not ros::ok() or t_err) {
    ROS_ERROR_STREAM_COND_NAMED(ros::ok(), "tm_socket_ros_heartbeat_timer", "Timer Error: " << t_err.message());
    this->stop();
  } else {
    this->ros_heartbeat_timer_.expires_from_now(HEARTBEAT_INTERVAL);
    this->ros_heartbeat_timer_.async_wait(boost::bind(&TMRobotListener::check_ros_heartbeat, this, error));
  }
}

/**
 * @details This handler always tries to reconnect to the server, unless ROS is stopped.
 *
 *           After the connection is established, obtain first message when entering listener node, this message serves
 *           as a signal to tell which handler should handle the work
 */
void TMRobotListener::handle_connection(boost::system::error_code const &t_err) noexcept {
  using namespace boost::asio::placeholders;

  if (not ros::ok()) {
    return;
  }

  if (t_err) {
    ROS_ERROR_STREAM_THROTTLE_NAMED(1.0, "tm_socket_connection",
                                    "Connection error, reason: " << t_err.message() << ", retrying...");

    this->listener_.async_connect(this->tm_robot_, boost::bind(&TMRobotListener::handle_connection, this, error));
  } else {
    ROS_INFO_STREAM_NAMED("tm_socket_connection", "Connection success, waiting for server response");

    boost::asio::async_read_until(this->listener_, this->input_buffer_, MESSAGE_END_BYTE,
                                  boost::bind(&TMRobotListener::handle_read, this, error, bytes_transferred));
  }
}

std::string TMRobotListener::extract_buffer_data(boost::asio::streambuf &t_buffer,
                                                 size_t const t_byte_to_extract) noexcept {
  if (t_byte_to_extract > 0) {
    auto const buffer_begin = boost::asio::buffers_begin(t_buffer.data());
    std::string const result{buffer_begin, buffer_begin + t_byte_to_extract};
    t_buffer.consume(t_byte_to_extract);

    return result;
  }

  return std::string{""};
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
void TMRobotListener::handle_read(boost::system::error_code const &t_err, size_t const t_byte_transfered) noexcept {
  using namespace boost::asio::placeholders;

  if (not ros::ok()) {
    return;
  }

  if (not t_err) {
    if (t_byte_transfered > 0) {
      auto const result        = this->extract_buffer_data(this->input_buffer_, t_byte_transfered);
      auto const parsed_result = ::get_tokenized_result(result);
      ROS_INFO_STREAM("Received: " << ::strip_crlf(result));

      auto const id     = *boost::next(parsed_result.begin(), ID_INDEX);
      auto const header = *parsed_result.begin();
      if (not this->current_task_handler_) {
        if (header == motion_function::TMSCT and id == TMR_INIT_MSG_ID) {
          // assign current handler to the one that satisfies the condition (match message)
          auto const data = std::vector<std::string>{boost::next(parsed_result.begin(), SCRIPT_START_INDEX),
                                                     boost::prior(parsed_result.end(), 1)};
          ROS_INFO_STREAM("In Listener node, node message: " << data[0]);
          auto const predicate = [&data](auto const &t_handler) {
            return t_handler->start_task_handling(data) == Decision::Accept;
          };
          auto const matched = std::find_if(this->task_handlers_.begin(), this->task_handlers_.end(), predicate);

          auto const cmd = [&]() {
            if (matched != this->task_handlers_.end()) {
              this->current_task_handler_ = *matched;
              return this->current_task_handler_->generate_request();
            }

            ROS_WARN_NAMED("tm_listener_node", "tm_listener_node doesn't find any handler satisfies the condition.");
            return this->default_task_handler_->generate_request();
          }();
          this->output_buffer_ = cmd->to_str();

          ROS_INFO_STREAM_NAMED("tm_listen_node", "Write msg: " << ::strip_crlf(this->output_buffer_));
          boost::asio::async_write(this->listener_, boost::asio::buffer(this->output_buffer_),
                                   boost::bind(&TMRobotListener::handle_write, this, error, bytes_transferred));
        }
      } else {
        this->current_task_handler_->handle_response(parsed_result);
      }

      // initiate another read process
      boost::asio::async_read_until(this->listener_, this->input_buffer_, MESSAGE_END_BYTE,
                                    boost::bind(&TMRobotListener::handle_read, this, error, bytes_transferred));
    }
  } else {
    ROS_ERROR_STREAM_NAMED("tm_listener_node", "Read Error: " << t_err.message());
    ROS_ERROR_STREAM_NAMED("tm_socket_connection", "Read Error detected, reconnecting...");

    this->reconnect();
  }
}

void TMRobotListener::handle_write(boost::system::error_code const &t_err, size_t const t_byte_writtened) noexcept {
  using namespace boost::asio::placeholders;

  if (not ros::ok()) {
    return;
  }

  if (not t_err) {
    if (this->current_task_handler_) {
      auto const cmd       = this->current_task_handler_->generate_request();
      this->output_buffer_ = cmd->to_str();
      if (cmd->has_script_exit()) {
        this->current_task_handler_.reset();
      }

      ROS_INFO_STREAM_COND_NAMED(not this->output_buffer_.empty(), "tm_listen_node",
                                 "Write msg: " << ::strip_crlf(this->output_buffer_));
      boost::asio::async_write(this->listener_, boost::asio::buffer(this->output_buffer_),
                               boost::bind(&TMRobotListener::handle_write, this, error, bytes_transferred));
    }
  } else {
    ROS_ERROR_STREAM_NAMED("tm_listener_node", "Write Error: " << t_err.message());
    ROS_ERROR_STREAM_NAMED("tm_socket_connection", "Write Error detected, reconnecting...");

    this->reconnect();
  }
}

void TMRobotListener::listener_node() {
  using namespace boost::asio::placeholders;

  this->listener_.async_connect(this->tm_robot_, boost::bind(&TMRobotListener::handle_connection, this, error));
  this->ros_heartbeat_timer_.expires_from_now(HEARTBEAT_INTERVAL);
  this->ros_heartbeat_timer_.async_wait(boost::bind(&TMRobotListener::check_ros_heartbeat, this, error));

  try {
    this->io_service_.run();
  } catch (std::exception &e) {
    ROS_ERROR_STREAM_COND_NAMED(ros::ok(), "tm_listener_connection", "Exception: " << e.what());
  }
}

void TMRobotListener::start() {
  if (ros::ok()) {
    this->listener_node_thread_ = boost::thread{&TMRobotListener::listener_node, this};
  }

  ros::spin();

  if (this->listener_node_thread_.joinable()) {
    this->listener_node_thread_.join();
  }
}

void TMRobotListener::stop() noexcept {
  boost::system::error_code ignore_error_code;
  this->listener_.close(ignore_error_code);
  this->ros_heartbeat_timer_.cancel();
}

void TMRobotListener::reconnect() noexcept {
  using namespace boost::asio::placeholders;

  this->current_task_handler_.reset();
  boost::system::error_code ignore_error_code;
  this->listener_.close(ignore_error_code);
  this->listener_.async_connect(this->tm_robot_, boost::bind(&TMRobotListener::handle_connection, this, error));
}

}  // namespace tm_robot_listener