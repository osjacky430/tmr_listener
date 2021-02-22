#ifndef TM_ERROR_HANDLER_HPP__
#define TM_ERROR_HANDLER_HPP__

#include "tmr_listener_handle/tmr_listener_handle.hpp"

namespace tm_error_handler {

class TMErrorHandler final : public tm_robot_listener::ListenerHandle {
  enum class HandlerState { ChangePayload, MoveToHome, Waiting, Finish, Error };
  HandlerState state_ = HandlerState::ChangePayload;

  friend HandlerState& operator++(HandlerState& t_state) noexcept {
    t_state = t_state == HandlerState::Finish
                ? HandlerState::Finish
                : static_cast<HandlerState>(static_cast<std::underlying_type_t<HandlerState>>(t_state) + 1);
    return t_state;
  }

  tm_robot_listener::Variable<float> payload_{"payload"};
  tm_robot_listener::Variable<std::array<float, 6>> targetP1{"targetP1"};
  tm_robot_listener::Variable<std::array<float, 6>> targetP2{"targetP2"};

  using MessageType = tm_robot_listener::motion_function::BaseHeaderProductPtr;

 protected:
  MessageType generate_cmd(MessageStatus const t_prev_response) override {
    using namespace tm_robot_listener;
    using namespace tm_robot_listener::motion_function;
    using namespace std::string_literals;
    if (t_prev_response == MessageStatus::Responded) {
      switch (this->state_) {
        case HandlerState::ChangePayload:
          return TMSCT << ID{"ChangePayload"} << declare(this->payload_, 0.0f) << ChangeLoad(this->payload_) << End();
        case HandlerState::MoveToHome:
          return TMSCT << ID{"MoveToHome"} << declare(targetP1, std::array<float, 6>{205, -35, 125, 0, 90, 0})
                       << PTP("JPP"s, targetP1, 100, 200, 100, true) << QueueTag(1)
                       << declare(targetP2, std::array<float, 6>{90, -35, 125, 0, 90, 0})
                       << PTP("JPP"s, targetP2, 100, 200, 100, true) << QueueTag(2, 1) << End();
        case HandlerState::Waiting:
          // return TMSCT << ID{"1"} << Pause() << End();
          // return TMSTA << QueueTagDone(1) << End();
          return empty_command_list();
        case HandlerState::Finish:
        case HandlerState::Error:
          return TMSCT << ID{"1"} << ScriptExit();
      }
    }

    return empty_command_list();
  }

  void response_msg(tm_robot_listener::TMSCTResponse const& t_resp) override {
    std::cout << t_resp.id_ << '\n' << boost::lexical_cast<std::string>(t_resp.script_result_) << '\n';
    for (auto const l : t_resp.abnormal_line_) {
      std::cout << l << '\n';
    }

    if (this->state_ != HandlerState::Waiting) {
      ++this->state_;
    }
  }

  void response_msg(tm_robot_listener::TMSTAResponse const& t_resp) override {
    std::cout << t_resp.subcmd_ << '\n';
    for (auto const l : t_resp.data_) {
      std::cout << l << '\n';
    }

    ++this->state_;
  }

  tm_robot_listener::Decision start_task(std::vector<std::string> const& t_name) override {
    if (t_name[0] == "VisionFail" or t_name[0] == "UltrasonicFail") {
      this->state_ = HandlerState::ChangePayload;
      return tm_robot_listener::Decision::Accept;
    }

    return tm_robot_listener::Decision::Ignore;
  }
};

}  // namespace tm_error_handler

#endif