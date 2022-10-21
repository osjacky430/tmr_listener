#include <boost/range/adaptor/transformed.hpp>
#include <boost/thread/scoped_thread.hpp>
#include <boost/variant.hpp>
#include <ros/ros.h>

#include <functional>
#include <numeric>

#include "tmr_listener/tmr_listener.hpp"
#include "tmr_utility/tmr_lambda_visitor.hpp"

namespace {

#if ROSCONSOLE_MIN_SEVERITY != ROSCONSOLE_SEVERITY_NONE

inline auto strip_crlf(std::string const &t_input) noexcept {
  return t_input.size() < 2 ? t_input : std::string{t_input.begin(), t_input.end() - 2};
}

#endif

}  // namespace

namespace tmr_listener {

class TMRobotListener::PacketVisitor : public boost::static_visitor<> {
  /**
   * @note Every object passed as a raw pointer (or iterator) is assumed to be owned by the caller, so that its lifetime
   *       is handled by the caller. Viewed another way: ownership transferring APIs are relatively rare compared to
   *       pointer-passing APIs, so the default is "no ownership transfer."
   */
  TMRobotTCP *const tcp_comm_;
  TMTaskHandler &current_task_handler_;
  TMRPluginManagerBase const *const plugin_manager_;
  ros::Publisher sub_cmd_pub_;

  static constexpr auto PUBLISHER_QUEUE_SIZE = 5;

 public:
  explicit PacketVisitor(TMRobotListener *const t_tmr_listener, ros::NodeHandle t_nh)
    : boost::static_visitor<>(),
      tcp_comm_{&t_tmr_listener->tcp_comm_},
      current_task_handler_{t_tmr_listener->current_task_handler_},
      plugin_manager_{t_tmr_listener->plugin_manager_.get()},
      sub_cmd_pub_{t_nh.advertise<SubCmdDataMsg>("subcmd_90_99", PUBLISHER_QUEUE_SIZE)} {}

  template <typename T>
  struct skip {
    void operator()(T const & /*unused*/) const noexcept {}
  };

  void poll_for_current_handler(TMSCTEnterNodeMsg const &t_resp) const;

  void check_reset_needed(CPERRResponse const &t_resp) const noexcept {
    if (t_resp.err_ == ErrorCode::NotInListenNode) {
      this->current_task_handler_.reset();
    }
  }

  void broadcast_data_msg(TMSTAResponse::DataMsg const &t_resp) const noexcept {
    auto const &plugins             = this->plugin_manager_->get_all_plugins();
    auto const send_subcmd_data_msg = [t_resp](auto &&t_plugin) { t_plugin->handle_response(t_resp); };
    std::for_each(plugins.begin(), plugins.end(), send_subcmd_data_msg);

    SubCmdDataMsg msg;
    msg.channel = t_resp.cmd_;
    msg.value   = t_resp.data_;
    this->sub_cmd_pub_.publish(msg);
  }

  template <typename T>
  void operator()(T const &t_packet) const {
    auto const default_skip = [](auto const & /* unused */) { /* skip the rest */ };
    auto const to_visit     = as_variant(t_packet.data_.resp_);
    if (this->current_task_handler_) {
      auto const notifier = [this](auto const &t_resp) { this->current_task_handler_->handle_response(t_resp); };
      auto const visitor  = make_lambda_visitor(notifier, skip<TMSCTEnterNodeMsg>{}, skip<TMSTAResponse::DataMsg>{});
      boost::apply_visitor(visitor, to_visit);
    } else {
      auto const find_handler = [this](TMSCTEnterNodeMsg const &t_msg) { this->poll_for_current_handler(t_msg); };
      auto const visitor      = make_lambda_visitor(find_handler, default_skip);
      boost::apply_visitor(visitor, to_visit);
    }

    auto const cperr        = [this](CPERRResponse const &t_resp) { this->check_reset_needed(t_resp); };
    auto const data_msg     = [this](TMSTAResponse::DataMsg const &t_resp) { this->broadcast_data_msg(t_resp); };
    auto const post_process = make_lambda_visitor(cperr, data_msg, default_skip);
    boost::apply_visitor(post_process, to_visit);
  }
};

/**
 * @details Since TM robot sends TMSCT message only during listen node, we can assume that if there is no task handler
 *          currently, the incoming TMSCT message can only be the first message sent when entering listen node. With
 *          that said, one thing we need to do is to make sure if we exit script, e.g., by sending ScriptExit(),
 *          current_task_handler_ is reset.
 *
 *          If there is no handler that is willing to handle the current listen node, then default_task_handler_ will
 *          generate the response, sending ScriptExit() immediately to TM robot.
 */
void TMRobotListener::PacketVisitor::poll_for_current_handler(TMSCTEnterNodeMsg const &t_resp) const {
  if (t_resp.id_ == TMRobotListener::TMR_INIT_MSG_ID) {
    ROS_INFO_STREAM("In Listener node, node message: " << t_resp.msg_);

    auto const find_result = this->plugin_manager_->find_task_handler(t_resp.msg_);
    if (not find_result) {
      ROS_FATAL_STREAM("plugin_manager return null pointer in find_task_handler, exit program");
      throw std::runtime_error("plugin_manager return null pointer in find_task_handler");
    }

    auto const response_cmd = find_result->generate_request();
    if (not response_cmd->has_script_exit()) {  // may be default handler, reset immediately
      this->current_task_handler_ = find_result;
    }

    ROS_INFO_STREAM_NAMED("tm_listen_node", "Write msg: " << ::strip_crlf(response_cmd->to_str()));
    this->tcp_comm_->write(response_cmd->to_str());
  }
}

}  // namespace tmr_listener

namespace tmr_listener {

TMTaskHandlerArray_t RTLibPluginManager::load_plugins() {
  using boost::adaptors::transformed;

  auto const plugin_transformer = [this](auto const &t_name) { return this->class_loader_.createInstance(t_name); };
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

TMRobotListener::ListenData TMRobotListener::parse(std::string t_input) noexcept {
  using namespace boost::spirit::qi;
  using boost::spirit::ascii::space;

  ListenData ret_val;
  [[gnu::unused]] bool const res = phrase_parse(t_input.begin(), t_input.end(), parsing_rule(), space, ret_val);
  assert(res);  // res will be true like 99.999999% of time, if not, either the parsing rule is bad, or input is
                // corrupted, which should be checked before enter this function (@todo implement the check)

  return ret_val;
}

TMRobotListener::TMRobotListener(std::string const &t_ip_addr, ros::NodeHandle const &t_nh,
                                 TMRPluginManagerBasePtr t_plugin_manager) noexcept
  : tcp_comm_{TMRobotTCP::Callback{
                [this](auto &&t_ph) { this->receive_tm_msg_callback(std::forward<decltype(t_ph)>(t_ph)); },
                [this](auto &&t_ph) { this->finished_transfer_callback(std::forward<decltype(t_ph)>(t_ph)); },
                [this]() { this->disconnected_callback(); }},
              LISTENER_PORT, t_ip_addr},
    plugin_manager_{std::move(t_plugin_manager)},
    visitor_{std::make_unique<PacketVisitor>(this, t_nh)} {}

TMRobotListener::~TMRobotListener() = default;

/**
 * @todo  We don't need to parse the input in one step, maybe parse the input in mulitple step, see TMSVR parsing
 */
void TMRobotListener::receive_tm_msg_callback(std::string const &t_input) noexcept {
  ROS_INFO_STREAM("Received: " << ::strip_crlf(t_input));

  auto const &result = parse(t_input);
  boost::apply_visitor(*this->visitor_, result);
}

void TMRobotListener::finished_transfer_callback(size_t const /*t_byte_writtened*/) noexcept {
  if (this->current_task_handler_) {
    auto const cmd = this->current_task_handler_->generate_request();

#if CURRENT_TMFLOW_VERSION_GE(1, 82, 0000)
    ROS_WARN_COND_NAMED(cmd->use_priority_cmd() and cmd->cmd_size() > 1, "tm_listen_node",
                        "Priority command will ignore all motion functions in the same packet");
#endif

    if (cmd->has_script_exit()) {
      this->current_task_handler_.reset();
    }

    auto const str = cmd->to_str();
    ROS_INFO_STREAM_COND_NAMED(not str.empty(), "tm_listen_node", "Write msg: " << ::strip_crlf(str));
    this->tcp_comm_.write(str);
  }
}

void TMRobotListener::disconnected_callback() {
  auto const &plugins = this->plugin_manager_->get_all_plugins();
  std::for_each(plugins.cbegin(), plugins.cend(), [](auto &&t_plugin) { t_plugin->handle_disconnect(); });
  if (this->current_task_handler_) {
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