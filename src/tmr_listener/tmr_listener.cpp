#include <boost/asio/buffer.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/tokenizer.hpp>

#include <chrono>
#include <functional>
#include <numeric>

#include "tmr_listener/tmr_listener.hpp"

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

namespace tmr_listener {

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
void TMRobotListener::parse_msg(std::string const &t_input) noexcept {
  auto const parsed_result = ::get_tokenized_result(t_input);
  ROS_INFO_STREAM("Received: " << ::strip_crlf(t_input));

  auto const id     = *boost::next(parsed_result.begin(), ID_INDEX);
  auto const header = parsed_result.front();
  if (not this->current_task_handler_) {
    if (header == TMSCT and id == TMR_INIT_MSG_ID) {
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

      ROS_INFO_STREAM_NAMED("tm_listen_node", "Write msg: " << ::strip_crlf(cmd->to_str()));
      this->tcp_comm_.write(cmd->to_str());
    }
  } else {
    this->current_task_handler_->handle_response(parsed_result);
  }
}

/**
 * @details
 */
void TMRobotListener::finished_transfer_callback(size_t const /*t_byte_writtened*/) noexcept {
  if (this->current_task_handler_) {
    auto const cmd = this->current_task_handler_->generate_request();
    auto const str = cmd->to_str();
    if (cmd->has_script_exit()) {
      this->current_task_handler_.reset();
    }

    ROS_INFO_STREAM_COND_NAMED(not str.empty(), "tm_listen_node", "Write msg: " << ::strip_crlf(str));
    this->tcp_comm_.write(str);
  }
}

TMRobotListener::TMTaskHandlerArray_t TMRobotListener::get_all_plugins() {
  using boost::adaptors::transformed;
  auto const plugin_transformer = [this](auto const &t_name) { return this->class_loader_.createInstance(t_name); };
  auto const plugin_names       = this->private_nh_.param("listener_handles", std::vector<std::string>{});
  ROS_DEBUG_STREAM_NAMED("tmr_listener", "plugin num: " << plugin_names.size());

  auto const plugins = plugin_names | transformed(plugin_transformer);
  return TMTaskHandlerArray_t{plugins.begin(), plugins.end()};
}

void TMRobotListener::start() {
  while (not ros::ok()) {
  }

  auto thread = boost::thread{boost::bind(&TMRobotTCP::start_tcp_comm, boost::ref(this->tcp_comm_))};
  ros::spin();

  if (thread.joinable()) {
    thread.join();
  }
}

}  // namespace tmr_listener