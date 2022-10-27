#include <boost/program_options.hpp>
#include <ros/ros.h>

#include "../src/tmr_cmd_line_option.hpp"
#include "tmr_listener/tmr_listener.hpp"

inline void run_program(int argc, char **argv) {
  using namespace boost::program_options;
  auto const common_opt = tmr_listener::common_option();

  variables_map opt_map;
  store(parse_command_line(argc, argv, common_opt), opt_map);

  if (opt_map.count("help") != 0) {
    std::cout << common_opt << '\n';
    return;
  }

  auto const ip = opt_map["ip"].as<std::string>();
  ROS_INFO_STREAM("Prepare connection: " << ip);

  tmr_listener::TMRobotListener listener_node{ip};
  listener_node.start();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "tmr_listener");
  ros::NodeHandle const nh{"/tmr_listener"};

  try {
    run_program(argc, argv);
  } catch (std::exception &e) {
    ROS_ERROR_STREAM("Error encountered: " << e.what());
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}