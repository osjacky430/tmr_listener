#include <boost/program_options.hpp>  // IWYU pragma: keep

#include "tm_robot_listener/tm_robot_listener.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "tm_robot_listener");
  ros::NodeHandle nh{"/tm_robot_listener"};  // /tm_robot_manager

  using namespace boost::program_options;
  options_description listener_opt{"Listener node options"};
  listener_opt.add_options()                     //
    ("help", "Show this help message and exit")  //
    ("ip", value<std::string>()->default_value(tm_robot_listener::TMRobotListener::DEFAULT_IP_ADDRESS),
     "IP address of the TM robot, default 192.168.1.2")  //
    ("verbose", "Show listener node debug message");

  variables_map opt_map;
  store(parse_command_line(argc, argv, listener_opt), opt_map);

  if (opt_map.count("help")) {
    std::cout << listener_opt << '\n';
    return 0;
  }

  auto const ip = opt_map["ip"].as<std::string>();

  tm_robot_listener::TMRobotListener listener_node{ip};
  listener_node.start();

  return 0;
}