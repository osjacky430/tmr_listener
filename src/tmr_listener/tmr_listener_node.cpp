#include <boost/program_options.hpp>  // IWYU pragma: keep
#include <ros/ros.h>

#include "../src/tmr_cmd_line_option.hpp"
#include "tmr_listener/tmr_listener.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "tmr_listener");
  ros::NodeHandle nh{"/tmr_listener"};

  using namespace boost::program_options;
  auto const common_opt = tmr_listener::common_option();

  variables_map opt_map;
  store(parse_command_line(argc, argv, common_opt), opt_map);

  if (opt_map.count("help") != 0) {
    std::cout << common_opt << '\n';
    return EXIT_SUCCESS;
  }

  auto const ip = opt_map["ip"].as<std::string>();
  ROS_INFO_STREAM("Prepare connection: " << ip);

  tmr_listener::TMRobotListener listener_node{ip};
  listener_node.start();

  return EXIT_SUCCESS;
}