#include "test_double/fake_server.hpp"
#include "test_double/mock_handler.hpp"
#include "tmr_test_constant.hpp"

#include <gtest/gtest.h>

using namespace ::testing;

class ListenNodeServerTest : public Test {
 protected:
  tmr_listener::fake_impl::ListenNodeServer fake_server;

  ros::NodeHandle nh{"tmr_listener"};
  ros::Subscriber sub = [this]() {  // wait until subscriber did subscribe to sth
    auto ret_val = nh.subscribe("subcmd_90_99", 0, &ListenNodeServerTest::callback, this);
    for (; ros::ok() && ret_val.getNumPublishers() == 0; ros::Duration(0.2).sleep()) {
    }
    return ret_val;
  }();
  tmr_listener::SubCmdDataMsg received_data_msg;

  void SetUp() override { this->fake_server.start(); }

  void TearDown() override { this->fake_server.stop(); }

  void callback(tmr_listener::SubCmdDataMsgConstPtr const& t_ptr) noexcept { this->received_data_msg = *t_ptr; }
};

TEST_F(ListenNodeServerTest, RecievedCPERRNotInListenNodeWillResetCurrentHandler) {
  using namespace tmr_listener;
  using namespace std::chrono_literals;

  // enter listen node, first command arrived
  auto const response = this->fake_server.enter_listen_node(FIRST_TEST_NAME, fake_impl::WAIT_FOREVER);
  ASSERT_TRUE(not response.empty());

  this->fake_server.send_error(ErrorCode::NotInListenNode);
  this->fake_server.response_ok_msg(ID{"expect_not_called"});  // expecting no response from client
}

TEST_F(ListenNodeServerTest, ScriptExitCalledIfNoHandlerReturnAccept) {
  using namespace tmr_listener;
  using namespace std::chrono_literals;

  // enter listen node, first command arrived
  auto const response = this->fake_server.enter_listen_node(SECOND_TEST_NAME, fake_impl::WAIT_FOREVER);
  EXPECT_THAT(response, HasSubstr("ScriptExit()"));

  // at this point current_task_handler_ should be reset, sending any msg to client will have no response
  this->fake_server.response_ok_msg(ID{"expect_not_called"});  // expecting no response from client
}

TEST_F(ListenNodeServerTest, SendTMSTADataMsgWillPublishData) {
  using namespace std::chrono_literals;
  using namespace std::string_literals;

  constexpr auto ROS_PROCESS_TIME = 2;

  this->fake_server.send_tmsta_data_msg(90, "{-1.230881,0.9187012,-252.5782,90.27595,-0.2474212,0.09009771}"s);
  ros::Duration(ROS_PROCESS_TIME).sleep();  // wait for ros to process
  ros::spinOnce();

  EXPECT_EQ(this->received_data_msg.channel, 90);
  EXPECT_EQ(this->received_data_msg.value, "{-1.230881,0.9187012,-252.5782,90.27595,-0.2474212,0.09009771}"s);
}