#ifndef TM_ERROR_HANDLER_HPP_
#define TM_ERROR_HANDLER_HPP_

#include <algorithm>

#include <ros/ros.h>
#include <ros/service.h>
#include <std_msgs/Float32MultiArray.h>

#include <boost/tokenizer.hpp>

#include "tmr_ext_script/tmr_parameterized_object.hpp"
#include "tmr_listener/EthernetSlaveCmd.h"
#include "tmr_listener/JsonDataArray.h"
#include "tmr_listener/tmr_listener_handle.hpp"

namespace tm_error_handler {

static inline auto get_tokenized_result(std::string const& t_result) noexcept {
  auto const tok_res = boost::tokenizer<boost::char_separator<char>>(t_result, boost::char_separator<char>(","));
  auto const ret_val = std::vector<std::string>{tok_res.begin(), tok_res.end()};
  return ret_val;
}

class TMErrorHandler final : public tmr_listener::ListenerHandle {
  enum class HandlerState { ChangePayload, MoveToHome, Waiting, Finish, Error };

  void data_table_cb(tmr_listener::JsonDataArray::ConstPtr const& t_data) {
    auto joint_torque = std::find_if(t_data->data.begin(), t_data->data.end(),
                                     [](auto const& t_elem) { return t_elem.item == R"("Joint_Torque")"; });
    if (joint_torque != t_data->data.end()) {
      std_msgs::Float32MultiArray pub_val;
      auto const torque_val = tmr_listener::parse_as<float, 6>(joint_torque->value);
      std::transform(torque_val.begin(), torque_val.end(), std::back_inserter(pub_val.data),
                     [](auto const& t_in) { return t_in / 1000; });
      this->pub_joint_torque_.publish(pub_val);
    }

    if (this->state_ == HandlerState::Finish) {
      auto project_run = std::find_if(t_data->data.begin(), t_data->data.end(),
                                      [](auto const& t_elem) { return t_elem.item == R"("Project_Run")"; });
      if (project_run != t_data->data.end() and not tmr_listener::parse_as<bool>(project_run->value)) {
        tmr_listener::EthernetSlaveCmd cmd;
        cmd.request.id = "StartProject";
        cmd.request.item_list.emplace_back("Stick_PlayPause");
        cmd.request.value_list.emplace_back("true");

        if (not ros::service::call("/tmr_eth_slave/tmsvr_cmd", cmd)) {
          ROS_ERROR_STREAM("service fail");
        } else if (cmd.response.res != "00,OK") {
          ROS_ERROR_STREAM(cmd.response.res);
        }

        this->state_ = HandlerState::ChangePayload;
      }
    }
  }

  HandlerState state_ = HandlerState::ChangePayload;

  friend HandlerState& operator++(HandlerState& t_state) noexcept {
    t_state = t_state == HandlerState::Finish
                ? HandlerState::Finish
                : static_cast<HandlerState>(static_cast<std::underlying_type_t<HandlerState>>(t_state) + 1);
    return t_state;
  }

  tmr_listener::Variable<int> var_listener{"var_listener"};
  tmr_listener::Variable<float> payload_{"payload"};
  tmr_listener::Variable<std::array<float, 6>> targetP1{"targetP1"};
  tmr_listener::Variable<std::array<float, 6>> targetP2{"targetP2"};
  int running_mission_ = 0;

  ros::NodeHandle nh_{"tmr_eth_slave"};
  ros::Subscriber parsed_data_table_{nh_.subscribe("parsed_data_table", 10, &TMErrorHandler::data_table_cb, this)};
  ros::Publisher pub_joint_torque_{nh_.advertise<std_msgs::Float32MultiArray>("joint_torque", 1)};

  using MessageType = tmr_listener::MessagePtr;

 protected:
  MessageType generate_cmd(MessageStatus const t_prev_response) override {
    using namespace tmr_listener;
    using namespace tmr_listener::motion_function;
    using namespace std::string_literals;
    if (t_prev_response == MessageStatus::Responded) {
      switch (this->state_) {
        case HandlerState::ChangePayload:
          // variable used without "declare" is considered to be project variable
          return TMSCT << ID{"ChangePayloadAndVarListener"} << declare(this->payload_, 0.0f)
                       << ChangeLoad(this->payload_) << (this->var_listener = 30001) << End();
        case HandlerState::MoveToHome: {
          auto const slot_num = (std::abs(this->running_mission_) / 100) % 10;

          auto cmd = TMSCT << ID{"MoveToHome"};
          if (slot_num == 1 or slot_num == 2) {
            cmd << declare(targetP1, Point["P328"].Value) << Line("CPP"s, targetP1, 50, 500, 100, true) << QueueTag(1)
                << declare(targetP2, std::array<float, 6>{90, -35, 125, 0, 90, 0})  // Point["Safety_front"]
                << PTP("JPP"s, targetP2, 50, 500, 100, true) << QueueTag(2, 1);
          } else if (slot_num == 3 or slot_num == 4) {
            cmd << Line("CPP"s, Point["safety_turn_b_left_buffer34"].Value, 50, 500, 100, true) << QueueTag(1)
                << PTP("CPP"s, Point["Safety_Back"].Value, 50, 500, 100, true) << QueueTag(2, 1);
          } else {
            ROS_WARN_ONCE("EQ port, not done yet");
            return cmd << ScriptExit();
          }

          return cmd << End();
        }
        case HandlerState::Waiting:
          return dummy_command_list("Waiting");
        case HandlerState::Finish:
          return TMSCT << ID{"Finish"} << ScriptExit();
        case HandlerState::Error:
          return TMSCT << ID{"Error"} << ScriptExit();
      }
    }

    return empty_command_list();
  }

  void response_msg(tmr_listener::TMSCTResponse const& t_resp) override {
    ROS_INFO_STREAM_NAMED("received_msg", "TMSCT response: ID=" << t_resp.id_ << ", Result=" << t_resp.script_result_);

    if (this->state_ != HandlerState::Waiting and t_resp.id_ != "Waiting" and t_resp.id_ != "MoveToHome") {
      ++this->state_;
    } else if (not t_resp.abnormal_line_.empty()) {
      ROS_WARN_STREAM_NAMED("received_msg", "TMSCT warning or error!");
      this->state_ = HandlerState::Error;
    }
  }

  void response_msg(tmr_listener::TMSTAResponse::Subcmd01 const& t_resp) override {
    ROS_INFO_STREAM_NAMED("received_msg", "TMSTA Subcmd01, data: " << t_resp.tag_number_);
    ++this->state_;
  }

  void response_msg(tmr_listener::CPERRResponse const& t_resp) override { this->state_ = HandlerState::Error; }

  tmr_listener::Decision start_task(std::string const& t_name) override {
    auto const tokenized = get_tokenized_result(t_name);
    if (/*t_name[0] == "VisionFail" or*/ tokenized[0] == "UltrasonicFail") {
      this->state_           = HandlerState::ChangePayload;
      this->running_mission_ = tmr_listener::parse_as<int>(tokenized[1]);
      return tmr_listener::Decision::Accept;
    }

    return tmr_listener::Decision::Ignore;
  }

 public:
  TMErrorHandler() = default;
};

}  // namespace tm_error_handler

#endif