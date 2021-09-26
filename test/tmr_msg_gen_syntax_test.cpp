#include "tmr_ext_script/tmr_motion_function.hpp"
#include "tmr_listener/tmr_listener_handle.hpp"

#define EXPECT_COMPILE_FAIL(expr)                                                                                  \
  class FailTest final : public tmr_listener::ListenerHandle {                                                     \
    using tm_msg_type = tmr_listener::MessagePtr;                                                                  \
                                                                                                                   \
   protected:                                                                                                      \
    tm_msg_type generate_cmd(MessageStatus const /**/) override {                                                  \
      using namespace tmr_listener;                                                                                \
      using namespace tmr_listener::motion_function;                                                               \
      return expr;                                                                                                 \
    }                                                                                                              \
                                                                                                                   \
    tmr_listener::Decision start_task(std::string const& /**/) override { return tmr_listener::Decision::Accept; } \
  }

#if defined(TEST_EXPRESSION)

EXPECT_COMPILE_FAIL(TEST_EXPRESSION);

#endif

int main(int /**/, char** /**/) { return EXIT_SUCCESS; }