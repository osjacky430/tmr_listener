#include "tmr_listener_handle/tmr_listener_handle.hpp"
#include "tmr_listener_handle/tmr_motion_function.hpp"

#define EXPECT_COMPILE_FAIL(expr)                                                             \
  class FailTest final : public tm_robot_listener::ListenerHandle {                           \
    using tm_msg_type = tm_robot_listener::motion_function::BaseHeaderProductPtr;             \
                                                                                              \
   protected:                                                                                 \
    tm_msg_type generate_cmd(MessageStatus const t_prev_response) override {                  \
      using namespace tm_robot_listener::motion_function;                                     \
      return expr;                                                                            \
    }                                                                                         \
                                                                                              \
    tm_robot_listener::Decision start_task(std::vector<std::string> const& t_name) override { \
      return tm_robot_listener::Decision::Accept;                                             \
    }                                                                                         \
  }

#if defined(TEST_EXPRESSION)

EXPECT_COMPILE_FAIL(TEST_EXPRESSION);

#endif

int main(int argc, char** argv) { return 0; }