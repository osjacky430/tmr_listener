#include "mock_handler.hpp"
#include "mock_handler_expectation.hpp"
#include "tmr_listener/tmr_listener.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "fake_listener_node");
  ros::NodeHandle nh{"/tmr_listener"};

  using namespace std::string_literals;
  using namespace tmr_listener;

  using TestMockPluginManager = InLibraryTMRPluginManager<PluginExpectationSetter>;
  TMRobotListener t{"127.0.0.1"s, nh, boost::make_shared<TestMockPluginManager>()};
  t.start();

  return EXIT_SUCCESS;
}