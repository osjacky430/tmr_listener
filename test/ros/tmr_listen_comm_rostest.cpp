#include "test_double/fake_server.hpp"
#include "test_double/mock_handler.hpp"
#include "tmr_test_constant.hpp"

#include <gtest/gtest.h>

using namespace ::testing;

class TMRServerTest : public Test {
 protected:
  tmr_listener::fake_impl::TMRServer fake_server;

  void SetUp() override {
    this->fake_server.start();
    this->fake_server.wait_until_connected();
  }

  void TearDown() override { this->fake_server.stop(); }
};

TEST_F(TMRServerTest, RecievedCPERRNotInListenNodeWillResetCurrentHandler) {
  using namespace tmr_listener;
  using namespace std::literals::chrono_literals;

  // enter listen node, first command arrived
  auto const response = this->fake_server.enter_listen_node(FIRST_TEST_NAME, fake_impl::WAIT_FOREVER);
  ASSERT_TRUE(not response.empty());

  this->fake_server.send_error(ErrorCode::NotInListenNode);
  this->fake_server.response_ok_msg(ID{"expect_not_called"});  // expecting no response from client
}

TEST_F(TMRServerTest, ScriptExitCalledIfNoHandlerReturnAccept) {
  using namespace tmr_listener;
  using namespace std::literals::chrono_literals;

  // enter listen node, first command arrived
  auto const response = this->fake_server.enter_listen_node(SECOND_TEST_NAME, fake_impl::WAIT_FOREVER);
  EXPECT_THAT(response, HasSubstr("ScriptExit()"));

  // at this point current_task_handler_ should be reset, sending any msg to client will have no response
  this->fake_server.response_ok_msg(ID{"expect_not_called"});  // expecting no response from client
}