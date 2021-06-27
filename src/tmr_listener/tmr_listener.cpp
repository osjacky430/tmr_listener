#include <boost/thread/scoped_thread.hpp>
#include <boost/variant.hpp>

#include <functional>
#include <numeric>

#include "tmr_listener/tmr_listener.hpp"

namespace {

inline auto strip_crlf(std::string const &t_input) noexcept {
  return t_input.size() < 2 ? t_input : std::string{t_input.begin(), t_input.end() - 2};
}

}  // namespace

namespace tmr_listener {

class TMRobotListener::PacketVisitor final : public boost::static_visitor<> {
  /**
   * @note Every object passed as a raw pointer (or iterator) is assumed to be owned by the caller, so that its lifetime
   *       is handled by the caller. Viewed another way: ownership transferring APIs are relatively rare compared to
   *       pointer-passing APIs, so the default is "no ownership transfer."
   */
  TMRobotTCP *tcp_comm_;
  TMTaskHandler &current_task_handler_;
  TMRPluginManagerBase const *const plugin_manager_;

 public:
  explicit PacketVisitor(TMRobotTCP *t_tcp_comm, TMTaskHandler &t_current_task_handler_,
                         TMRPluginManagerBase const *const t_plugin_manager)
    : boost::static_visitor<>(),
      tcp_comm_{t_tcp_comm},
      current_task_handler_{t_current_task_handler_},
      plugin_manager_{t_plugin_manager} {}

  template <typename T>
  void operator()(T const & /*unused*/) {
    throw std::invalid_argument("Unimplemented or unknown header, please help report issues.");
  }
};

template <>
void TMRobotListener::PacketVisitor::operator()(CPERRPacket const &t_packet) {
  if (this->current_task_handler_) {
    this->current_task_handler_->handle_response(CPERRResponse{t_packet.data_.err_});
    if (t_packet.data_.err_ == ErrorCode::NotInListenNode) {
      this->current_task_handler_.reset();
    }
  }
}

/**
 * @note  Maybe I will implement TMSTAPacket service in the future, by then, these if looks even more redundant cause it
 *        can only prevent us from using null pointer. Not like the else part can be used for other situation. For the
 *        situation metioned above, we can only determine the decision via id, where service have specific ID created
 *        by us
 */
template <>
void TMRobotListener::PacketVisitor::operator()(TMSTAPacket const &t_packet) {
  boost::apply_visitor(*this, t_packet.data_.resp_);
}

template <>
void TMRobotListener::PacketVisitor::operator()(TMSTAPacket::DataFrame::Subcmd00Resp const &t_packet) {
  if (this->current_task_handler_) {
    TMSTAResponse::Subcmd00 resp{boost::get<0>(t_packet), boost::get<1>(t_packet)};
    this->current_task_handler_->handle_response(resp);
  }
}

template <>
void TMRobotListener::PacketVisitor::operator()(TMSTAPacket::DataFrame::Subcmd01Resp const &t_packet) {
  if (this->current_task_handler_) {
    TMSTAResponse::Subcmd01 resp{boost::get<0>(t_packet), boost::get<1>(t_packet)};
    this->current_task_handler_->handle_response(resp);
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
 * @note    TM robot will send OK message even after ScriptExit()
 *
 * @todo    I would like to take if else away from these operator()s
 */
template <>
void TMRobotListener::PacketVisitor::operator()(TMSCTPacket const &t_tmsct) {
  auto const server_response = boost::get<TMSCTPacket::DataFrame::ServerResponse>(t_tmsct.data_.cmd_);
  if (not this->current_task_handler_) {
    if (t_tmsct.data_.id_ == TMRobotListener::TMR_INIT_MSG_ID) {
      auto const response_content = boost::get<std::string>(server_response);
      ROS_INFO_STREAM("In Listener node, node message: " << response_content);

      auto const find_result  = this->plugin_manager_->find_task_handler(response_content);
      auto const response_cmd = find_result->generate_request();
      if (not response_cmd->has_script_exit()) {  // may be default handler, reset immediately
        this->current_task_handler_ = find_result;
      }

      ROS_INFO_STREAM_NAMED("tm_listen_node", "Write msg: " << ::strip_crlf(response_cmd->to_str()));
      this->tcp_comm_->write(response_cmd->to_str());
    }
  } else {
    auto const response_content = boost::get<TMSCTPacket::DataFrame::ScriptResult>(server_response);
    TMSCTResponse const resp{t_tmsct.data_.id_, response_content.result_, response_content.abnormal_lines_};
    this->current_task_handler_->handle_response(resp);
  }
}

}  // namespace tmr_listener

namespace tmr_listener {

TMTaskHandlerArray_t RTLibPluginManager::load_plugins() {
  using boost::adaptors::transformed;

  auto const plugin_transformer = [this](auto const &t_name) { return class_loader_.createInstance(t_name); };
  auto const plugin_names       = ros::param::param("/tmr_listener/listener_handles", std::vector<std::string>{});
  ROS_DEBUG_STREAM_NAMED("tmr_plugin_manager", "plugin num: " << plugin_names.size());

  auto const plugins = plugin_names | transformed(plugin_transformer);
  return TMTaskHandlerArray_t{plugins.begin(), plugins.end()};
}

TMTaskHandler RTLibPluginManager::find_task_handler(std::string const &t_input) const noexcept {
  auto const predicate = [&t_input](auto const &t_handler) {
    return t_handler->start_task_handling(t_input) == Decision::Accept;
  };

  auto const matched = std::find_if(this->all_plugins_.begin(), this->all_plugins_.end(), predicate);
  if (matched == this->all_plugins_.end()) {
    ROS_WARN_STREAM_NAMED("tmr_plugin_manager", "No suitable handler to take care of current task, returning default");
    return this->default_handler_;
  }

  return *matched;
}

/**
 * @note  Even though I can make it a static class variable, I decided to create local static to make it
 *        similar to other parsing rule
 */
TMRobotListener::TMRListenDataParser TMRobotListener::parsing_rule() noexcept {
  static TMRListenDataParser parser = TMSCTPacket::parsing_rule() | TMSTAPacket::parsing_rule() |  //
                                      CPERRPacket::parsing_rule();
  return parser;
}

/**
 * @todo throw if not fully matched
 */
TMRobotListener::ListenData TMRobotListener::parse(std::string t_input) {
  using namespace boost::spirit::qi;
  using boost::spirit::ascii::space;

  ListenData ret_val;
  bool const res = phrase_parse(t_input.begin(), t_input.end(), parsing_rule(), space, ret_val);

  return ret_val;
}

/**
 * @todo  We don't need to parse the input in one step, maybe parse the input in mulitple step, see TMSVR parsing
 */
void TMRobotListener::receive_tm_msg_callback(std::string const &t_input) noexcept(noexcept(parse(t_input))) {
  ROS_INFO_STREAM("Received: " << ::strip_crlf(t_input));

  auto const result = parse(t_input);
  auto visitor      = PacketVisitor{&this->tcp_comm_, this->current_task_handler_, this->plugin_manager_.get()};
  boost::apply_visitor(visitor, result);
}

void TMRobotListener::finished_transfer_callback(size_t const /*t_byte_writtened*/) noexcept {
  if (this->current_task_handler_) {
    auto const cmd = this->current_task_handler_->generate_request();
    if (cmd->has_script_exit()) {
      this->current_task_handler_.reset();
    }

    auto const str = cmd->to_str();
    ROS_INFO_STREAM_COND_NAMED(not str.empty(), "tm_listen_node", "Write msg: " << ::strip_crlf(str));
    this->tcp_comm_.write(str);
  }
}

void TMRobotListener::disconnected_callback() noexcept {
  if (this->current_task_handler_) {
    this->current_task_handler_->handle_disconnect();
    this->current_task_handler_.reset();
  }
}

void TMRobotListener::start() {
  while (not ros::ok()) {
  }

  boost::scoped_thread<> thread{boost::thread{[&comm = this->tcp_comm_] { comm.start_tcp_comm(); }}};
  ros::spin();

  this->tcp_comm_.stop();
}

}  // namespace tmr_listener