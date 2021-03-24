#include "tmr_server_mock/tmr_server_mock.hpp"

int main(int /*argc*/, char** /*argv*/) {
  try {
    tmr_server::TMRobot robot{};
    robot.start();
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  int a;
  std::cin >> a;

  return 0;
}