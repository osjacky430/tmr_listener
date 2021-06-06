#include <algorithm>

#include "tmr_ethernet/tmr_eth_rw.hpp"
#include "tmr_ethernet/tmr_eth_slave.hpp"

namespace tmr_listener {

void TMRobotEthSlave::parse_input_msg(std::string const& t_input) noexcept {
  auto const parsed_data = TMSVR.parse(t_input);
  if (parsed_data.data_.mode_ == Mode::Json) {
    auto const raw_data = [](std::string const& t_to_set) {
      std_msgs::String ret_val;
      ret_val.data = t_to_set;
      return ret_val;
    }(parsed_data.data_.raw_content_);
    this->raw_data_table_pub_.publish(raw_data);

    auto const parsed_content = parsed_data.data_.parse_raw_content(parsed_data.data_.raw_content_);
    // *boost::get<std::vector<tmr_listener::TMSVRJsonData>>(&parsed_data.data_.data_);

    auto const processed_data = [](std::vector<tmr_listener::TMSVRJsonData> const& t_to_tf) {
      auto const tm_struct_to_ros_msg = [](auto const& t_tm_struct) {
        tmr_listener::JsonData ret_val;
        ret_val.item  = t_tm_struct.item_;
        ret_val.value = t_tm_struct.value_;
        return ret_val;
      };

      tmr_listener::JsonDataArray ret_val;
      std::transform(t_to_tf.begin(), t_to_tf.end(), std::back_inserter(ret_val.data), tm_struct_to_ros_msg);
      return ret_val;
    }(parsed_content);
    this->processed_data_table_pub_.publish(processed_data);
  } else {
    this->server_response_ = parsed_data.data_;
    this->responded        = true;
    this->response_signal_.notify_all();
  }
}

/**
 * @details Completion handlers only run in the context of a thread that has called io_service.run() no matter which
 *          thread called the asynchronous method. If you've only called io_service.run() in one thread then all
 *          completion handlers will execute serially in the context of that thread.
 */
bool TMRobotEthSlave::send_tmsvr_cmd(EthernetSlaveCmdRequest& t_req, EthernetSlaveCmdResponse& t_resp) {
  bool const is_read = t_req.value_list.empty();
  if (not is_read and t_req.value_list.size() != t_req.item_list.size()) {
    ROS_ERROR_STREAM("the size of item list must equal to the size of value list for write operation");
    return false;
  }

  if (t_req.id.empty()) {
    ROS_ERROR_STREAM("Empty ID found, @todo it will become auto generated if not entered in the future");
    return false;
  }

  try {
    auto const cmd = [&t_req, is_read]() {
      auto ret_cmd = TMSVR << ID{t_req.id};
      if (is_read) {
        std::for_each(t_req.item_list.begin(), t_req.item_list.end(),
                      [&ret_cmd](auto const& t_in) { ret_cmd << generate_read_req(t_in); });
      } else {
        for (std::size_t i = 0; i < t_req.item_list.size(); ++i) {
          ret_cmd << generate_write_req(t_req.item_list.at(i), t_req.value_list.at(i));
        }
      }

      auto const ret_val = ret_cmd << End();
      return ret_val->to_str();
    }();
    this->comm_.write(cmd);
  } catch (std::exception& e) {
    ROS_ERROR_STREAM(e.what());
    return false;
  }

  // @todo add timeout
  boost::unique_lock<boost::mutex> lock{this->rx_buffer_mutex_};
  auto const tm_responded = [this]() { return this->responded; };
  this->response_signal_.wait(lock, tm_responded);

  if (this->server_response_.mode_ == Mode::ServierResponse) {
    t_resp.res = this->server_response_.raw_content_;  // possible reason: response of request write, or error happened
  } else {
    t_req.value_list = [](std::string const& t_raw_content) {  // possible reason: request read
      std::vector<std::string> ret_val;
      auto const parsed = TMSVRPacket::DataFrame::parse_raw_content(t_raw_content);
      std::transform(parsed.begin(), parsed.end(), std::back_inserter(ret_val),
                     [](auto const& t_in) { return t_in.value_; });
      return ret_val;
    }(this->server_response_.raw_content_);
  }

  return true;
}

void TMRobotEthSlave::start() noexcept {
  while (not ros::ok()) {
  }

  auto thread = boost::thread{boost::bind(&TMRobotTCP::start_tcp_comm, boost::ref(this->comm_))};
  ros::spin();

  // make sure the service is shutdown
  this->responded = true;
  this->response_signal_.notify_all();

  if (thread.joinable()) {
    thread.join();
  }
}
}  // namespace tmr_listener
