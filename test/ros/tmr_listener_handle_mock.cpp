#include <gmock/gmock.h>

#include "test_double/mock_handler.hpp"
#include "test_double/mock_handler_expectation.hpp"
#include "tmr_test_constant.hpp"

using namespace ::testing;
using namespace std::string_literals;

namespace tmr_listener {

SET_EXPECTATION(0, t_first_mock) {
  EXPECT_CALL(t_first_mock, start_task(_)).WillRepeatedly(Return(Decision::Ignore));

  using MsgStatus_ = ListenerHandle::MessageStatus;
  EXPECT_CALL(t_first_mock, start_task(FIRST_TEST_NAME)).WillRepeatedly(Return(Decision::Accept));
  EXPECT_CALL(t_first_mock, generate_cmd(MsgStatus_::Responded)).WillRepeatedly(Return(dummy_command_list("dummy"s)));
  EXPECT_CALL(t_first_mock, generate_cmd(MsgStatus_::NotYetRespond)).WillRepeatedly(Return(empty_command_list()));

  EXPECT_CALL(t_first_mock, response_msg(CPERRResponse{ErrorCode::NotInListenNode})).Times(Exactly(1));
  EXPECT_CALL(t_first_mock, response_msg(TMSCTResponse{"expect_not_called"s, true})).Times(Exactly(0));
}

}  // namespace tmr_listener
