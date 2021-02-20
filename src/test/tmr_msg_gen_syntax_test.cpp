#include "tmr_listener_handle/tmr_listener_handle.hpp"
#include "tmr_listener_handle/tmr_motion_function.hpp"

#define EXPECT_COMPILE_FAIL(expr)                                                                                \
  class FailTest : public tm_robot_listener::ListenerHandle {                                                    \
   protected:                                                                                                    \
    tm_robot_listener::motion_function::BaseHeaderProductPtr generate_cmd(bool const t_prev_response) override { \
      using namespace tm_robot_listener::motion_function;                                                        \
      return expr;                                                                                               \
    }                                                                                                            \
                                                                                                                 \
    bool start_task(std::vector<std::string> const& t_name) { return true; }                                     \
  }

#if defined(TEST_EXPRESSION)

EXPECT_COMPILE_FAIL(TEST_EXPRESSION);

#endif

int main(int argc, char** argv) { return 0; }