#include "tmr_fake_server/fake_server.hpp"
#include "tmr_listener/EthernetSlaveCmd.h"

#include <boost/thread/scoped_thread.hpp>
#include <boost/thread/thread.hpp>

#include <gtest/gtest.h>
#include <ros/ros.h>

using namespace ::testing;
using namespace tmr_listener;
using namespace std::string_literals;

class EthernetServerTest : public Test {
 public:
  TMRobotServerComm fake_server{ETHERNET_SERVER_PORT};

 protected:
  static constexpr auto ETHERNET_SERVER_PORT = 5891;

  auto create_io_thread(std::string& t_resp, std::string const& t_to_client) {
    return boost::thread([&, this]() mutable {
      t_resp = this->fake_server.blocking_read();
      this->fake_server.blocking_write(t_to_client);
    });
  }

  void SetUp() override { ros::service::waitForService("/tmr_eth_slave/tmsvr_cmd"); }

  void TearDown() override { this->fake_server.stop(); }
};

/**
 *  @details  Due to the way ros service interface is designed, we can't get exactly the reason result in service call
 *            failure. Therefore, the test is written in the way that only one factor is changed at a time, first
 *            test fail case, then make a change (assuming that is the reason that makes the test fail), finally test
 *            with the rest of the input same and expect a success result
 */
TEST_F(EthernetServerTest, ServiceReturnFailIfNotConnected) {
  // fail: not connected
  EthernetSlaveCmd cmd;
  cmd.request = []() {  // read the value "TCP_Mass"
    EthernetSlaveCmdRequest ret_val;
    ret_val.id = "Q3";
    ret_val.item_list.emplace_back("TCP_Mass");
    return ret_val;
  }();
  EXPECT_FALSE(ros::service::call("/tmr_eth_slave/tmsvr_cmd", cmd));

  // start connection
  this->fake_server.start();

  // expect success
  std::string server_received_msg;
  std::string const& to_client = R"($TMSVR,39,Q3,13,[{"Item":"TCP_Mass","Value":0.0}],*40)"
                                 "\r\n"s;
  boost::scoped_thread<> t{this->create_io_thread(server_received_msg, to_client)};

  EXPECT_TRUE(ros::service::call("/tmr_eth_slave/tmsvr_cmd", cmd));
  EXPECT_EQ(server_received_msg, R"($TMSVR,27,Q3,13,[{"Item":"TCP_Mass"}],*3C)"
                                 "\r\n"s);

  ASSERT_EQ(cmd.response.value_list.size(), 1);
  EXPECT_EQ(cmd.response.value_list.at(0), "0.0");
}

TEST_F(EthernetServerTest, ServiceReturnFailIfEmptyID) {
  this->fake_server.start();

  // fail: no id specified
  EthernetSlaveCmd cmd;
  cmd.request.item_list.emplace_back("TCP_Mass");
  EXPECT_FALSE(ros::service::call("/tmr_eth_slave/tmsvr_cmd", cmd));

  std::string server_received_msg;
  std::string const& to_client = R"($TMSVR,39,Q3,13,[{"Item":"TCP_Mass","Value":0.0}],*40)"
                                 "\r\n"s;
  boost::scoped_thread<> t{this->create_io_thread(server_received_msg, to_client)};

  // specify id
  cmd.request.id = "Q3";

  // expect success
  EXPECT_TRUE(ros::service::call("/tmr_eth_slave/tmsvr_cmd", cmd));
  EXPECT_EQ(server_received_msg, R"($TMSVR,27,Q3,13,[{"Item":"TCP_Mass"}],*3C)"
                                 "\r\n"s);

  ASSERT_EQ(cmd.response.value_list.size(), 1);
  EXPECT_EQ(cmd.response.value_list.at(0), "0.0");
}

TEST_F(EthernetServerTest, ServiceReturnFailIfItemListAndValueListMismatched) {
  this->fake_server.start();

  // fail: item list and value list mismatched
  EthernetSlaveCmd cmd;
  cmd.request.id = "T9"s;
  cmd.request.item_list.emplace_back("Ctrl_DO0");
  cmd.request.value_list.emplace_back("1");

  cmd.request.item_list.emplace_back("Ctrl_DO1");
  cmd.request.value_list.emplace_back("0");

  cmd.request.item_list.emplace_back("g_ss");

  EXPECT_FALSE(ros::service::call("/tmr_eth_slave/tmsvr_cmd", cmd));

  // add the missing value to write back
  cmd.request.value_list.emplace_back(R"(["Hello","TM","Robot"])");

  std::string server_received_msg;
  std::string const& to_client = "$TMSVR,10,T9,0,00,OK,*16\r\n"s;
  boost::scoped_thread<> t{this->create_io_thread(server_received_msg, to_client)};

  // expect success
  EXPECT_TRUE(ros::service::call("/tmr_eth_slave/tmsvr_cmd", cmd));
  EXPECT_EQ(server_received_msg, R"($TMSVR,113,T9,3,[{"Item":"Ctrl_DO0","Value":1},)"
                                 R"({"Item":"Ctrl_DO1","Value":0},)"
                                 R"({"Item":"g_ss","Value":["Hello","TM","Robot"]}],*7C)"
                                 "\r\n"s);

  ASSERT_TRUE(cmd.response.value_list.empty());
  EXPECT_EQ(cmd.response.res, "00,OK");
}

TEST_F(EthernetServerTest, ServiceReturnFailIfInsertEmptyString) {
  this->fake_server.start();

  EthernetSlaveCmd cmd;
  cmd.request.id = "T9"s;
  cmd.request.item_list.emplace_back("Ctrl_DO0");
  cmd.request.value_list.emplace_back("1");

  cmd.request.item_list.emplace_back("Ctrl_DO1");
  cmd.request.value_list.emplace_back("0");

  cmd.request.item_list.emplace_back("g_ss");

  // fail: push empty string
  cmd.request.value_list.emplace_back("");

  EXPECT_FALSE(ros::service::call("/tmr_eth_slave/tmsvr_cmd", cmd));

  // modify last value to none empty
  cmd.request.value_list.back() = R"(["Hello","TM","Robot"])"s;

  std::string server_received_msg;
  std::string const& to_client = "$TMSVR,10,T9,0,00,OK,*16\r\n"s;
  boost::scoped_thread<> t{this->create_io_thread(server_received_msg, to_client)};

  EXPECT_TRUE(ros::service::call("/tmr_eth_slave/tmsvr_cmd", cmd));
  EXPECT_EQ(server_received_msg, R"($TMSVR,113,T9,3,[{"Item":"Ctrl_DO0","Value":1},)"
                                 R"({"Item":"Ctrl_DO1","Value":0},)"
                                 R"({"Item":"g_ss","Value":["Hello","TM","Robot"]}],*7C)"
                                 "\r\n"s);

  ASSERT_TRUE(cmd.response.value_list.empty());
  EXPECT_EQ(cmd.response.res, "00,OK");
}