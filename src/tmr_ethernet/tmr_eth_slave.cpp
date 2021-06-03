#include <algorithm>

#include "tmr_ethernet/tmr_eth_rw.hpp"
#include "tmr_ethernet/tmr_eth_slave.hpp"

namespace tmr_listener {

void TMRobotEthSlave::parse_input_msg(std::string const& t_input) noexcept {
  auto const parsed_data = TMSVR.parse(t_input);
  if (parsed_data.data_.mode_ == 3) {  // @todo change 3 to enum class or other thing
    {
      auto const pub_data = [](std::string const& t_to_set) {
        std_msgs::String ret_val;
        ret_val.data = t_to_set;
        return ret_val;
      }(parsed_data.data_.raw_content_);
      this->raw_data_table_pub_.publish(pub_data);
    }

    {
      auto const parsed_content = parsed_data.data_.parse_raw_content(parsed_data.data_.raw_content_);
      // *boost::get<std::vector<tmr_listener::TMSVRJsonData>>(&parsed_data.data_.data_);

      auto const pub_data = [](std::vector<tmr_listener::TMSVRJsonData> const& t_to_tf) {
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
      this->processed_data_table_pub_.publish(pub_data);
    }
  } else {
    // set server response
  }
}

void TMRobotEthSlave::start() noexcept {
  while (not ros::ok()) {
  }

  auto thread = boost::thread{boost::bind(&TMRobotTCP::start_tcp_comm, boost::ref(this->comm_))};
  ros::spin();

  if (thread.joinable()) {
    thread.join();
  }
}
}  // namespace tmr_listener
