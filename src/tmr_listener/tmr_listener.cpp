#include <boost/tokenizer.hpp>
#include <boost/variant.hpp>

#include <chrono>
#include <functional>
#include <numeric>

#include "tmr_listener/tmr_listener.hpp"

namespace {

inline auto strip_crlf(std::string const &t_input) noexcept {
  return t_input.size() < 2 ? t_input : std::string{t_input.begin(), t_input.end() - 2};
}

}  // namespace

namespace tmr_listener {

struct TMRobotListener::PacketVisitor final : public boost::static_visitor<void> {
  TMRobotListener *context_holder_;

  explicit PacketVisitor(TMRobotListener *t_ctx) : boost::static_visitor<void>(), context_holder_{t_ctx} {}

  template <typename T>
  void operator()(T const &) {
    ROS_INFO_STREAM("unknown");
  }
};

template <>
void TMRobotListener::PacketVisitor::operator()(CPERRPacket const &t_packet) {
  this->context_holder_->current_task_handler_->handle_response(CPERRResponse{t_packet.data_.err_});
}

template <>
void TMRobotListener::PacketVisitor::operator()(TMSTAPacket const &t_packet) {
  boost::apply_visitor(*this, t_packet.data_.resp_);
}

template <>
void TMRobotListener::PacketVisitor::operator()(TMSTAPacket::DataFrame::Subcmd00Resp const &t_packet) {
  TMSTAResponse::Subcmd00 resp{boost::get<0>(t_packet), boost::get<1>(t_packet)};
  this->context_holder_->current_task_handler_->handle_response(resp);
}

template <>
void TMRobotListener::PacketVisitor::operator()(TMSTAPacket::DataFrame::Subcmd01Resp const &t_packet) {
  TMSTAResponse::Subcmd01 resp{boost::get<0>(t_packet), boost::get<1>(t_packet)};
  this->context_holder_->current_task_handler_->handle_response(resp);
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
template <>
void TMRobotListener::PacketVisitor::operator()(TMSCTPacket const &t_tmsct) {
  TMRobotListener *&ctx = this->context_holder_;

  auto const server_response = boost::get<TMSCTPacket::DataFrame::ServerResponse>(t_tmsct.data_.cmd_);
  if (not ctx->current_task_handler_) {
    if (t_tmsct.data_.id_ == TMRobotListener::TMR_INIT_MSG_ID) {
      auto const response_content = boost::get<std::string>(server_response);
      ROS_INFO_STREAM("In Listener node, node message: " << response_content);
      auto const predicate = [&response_content](auto const &t_handler) {
        return t_handler->start_task_handling(response_content) == Decision::Accept;
      };
      auto const matched = std::find_if(ctx->task_handlers_.begin(), ctx->task_handlers_.end(), predicate);
      auto const cmd     = [&]() {
        if (matched != ctx->task_handlers_.end()) {
          ctx->current_task_handler_ = *matched;
          return ctx->current_task_handler_->generate_request();
        }

        ROS_WARN_NAMED("tm_listener_node", "tm_listener_node doesn't find any handler satisfies the condition.");
        return ctx->default_task_handler_->generate_request();
      }();

      ROS_INFO_STREAM_NAMED("tm_listen_node", "Write msg: " << ::strip_crlf(cmd->to_str()));
      ctx->tcp_comm_.write(cmd->to_str());
    }
  } else {
    auto const response_content = boost::get<TMSCTPacket::DataFrame::ScriptResult>(server_response);
    TMSCTResponse const resp{t_tmsct.data_.id_, response_content.result_, std::move(response_content.abnormal_lines_)};
    ctx->current_task_handler_->handle_response(resp);
  }
}

}  // namespace tmr_listener

namespace tmr_listener {

TMRobotListener::ListenData TMRobotListener::parse(std::string t_input) {
  using namespace boost::spirit::qi;
  using boost::spirit::ascii::space;

  static ParseRule<ListenData> parse_rule = TMSCTPacket::parsing_rule() | TMSTAPacket::parsing_rule() |  //
                                            CPERRPacket::parsing_rule();

  ListenData ret_val;
  bool const res = phrase_parse(t_input.begin(), t_input.end(), parse_rule, space, ret_val);

  return ret_val;
}

/**
 * @todo  We don't need to parse the input in one step, maybe parse the input in mulitple step, see TMSVR parsing
 * @todo  If ScriptExit() happened, this->current_task_handler_ will reset, result in bad_get for not-yet-responded
 *        message. Maybe the ID of the message needs to be traced. Not sure.
 */
void TMRobotListener::receive_tm_msg_callback(std::string const &t_input) noexcept {
  ROS_INFO_STREAM("Received: " << ::strip_crlf(t_input));

  auto const result = parse(t_input);
  auto visitor      = PacketVisitor{this};
  boost::apply_visitor(visitor, result);
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

/**
 * @details
 *
 */
void TMRobotListener::disconnected_callback() noexcept {
  if (this->current_task_handler_) {
    this->current_task_handler_->handle_disconnect();
    this->current_task_handler_.reset();
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