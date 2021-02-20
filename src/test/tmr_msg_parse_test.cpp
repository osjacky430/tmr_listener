#include <gtest/gtest.h>

#include "tmr_listener_handle/tmr_listener_handle.hpp"

class MsgParseTester final : public tm_robot_listener::ListenerHandle {
 public:
  tm_robot_listener::TMSCTResponse tmsct_resp_;
  tm_robot_listener::TMSTAResponse tmsta_resp_;
  tm_robot_listener::CPERRResponse cperr_resp_;

 protected:
  bool start_task(std::vector<std::string> const& t_data) override { return true; }

  tm_robot_listener::motion_function::BaseHeaderProductPtr generate_cmd(bool const) override {
    return tm_robot_listener::motion_function::BaseHeaderProductPtr{};
  }

  void response_msg(tm_robot_listener::TMSCTResponse const& t_resp) override { this->tmsct_resp_ = t_resp; }

  void response_msg(tm_robot_listener::TMSTAResponse const& t_resp) override { this->tmsta_resp_ = t_resp; }

  void response_msg(tm_robot_listener::CPERRResponse const& t_resp) override { this->cperr_resp_ = t_resp; }
};

TEST(MsgParseTest, ContentMatch) {
  MsgParseTester test;

  test.handle_response({"$TMSCT", "4", "2", "OK", "*5F"});
  EXPECT_TRUE(test.tmsct_resp_.abnormal_line_.empty());
  EXPECT_EQ(test.tmsct_resp_.id_, "2");
  EXPECT_TRUE(test.tmsct_resp_.script_result_);

  test.handle_response({"$TMSCT", "8", "2", "OK;2;3", "*52"});
  EXPECT_EQ(test.tmsct_resp_.abnormal_line_, (std::vector<int>{2, 3}));
  EXPECT_EQ(test.tmsct_resp_.id_, "2");
  EXPECT_TRUE(test.tmsct_resp_.script_result_);

  test.handle_response({"$TMSCT", "13", "3", "ERROR;1;2;3", "*3F"});
  EXPECT_EQ(test.tmsct_resp_.abnormal_line_, (std::vector<int>{1, 2, 3}));
  EXPECT_EQ(test.tmsct_resp_.id_, "3");
  EXPECT_FALSE(test.tmsct_resp_.script_result_);

  test.handle_response({"$TMSTA", "9", "00", "false", "*37"});
  EXPECT_EQ(test.tmsta_resp_.subcmd_, 0);
  EXPECT_EQ(test.tmsta_resp_.data_, (std::vector<std::string>{"false"}));

  test.handle_response({"$TMSTA", "15", "00", "true", "Listen1", "*79"});
  EXPECT_EQ(test.tmsta_resp_.subcmd_, 0);
  EXPECT_EQ(test.tmsta_resp_.data_, (std::vector<std::string>{"true", "Listen1"}));

  test.handle_response({"$TMSTA", "10", "01", "08", "true", "*6D"});
  EXPECT_EQ(test.tmsta_resp_.subcmd_, 1);
  EXPECT_EQ(test.tmsta_resp_.data_, (std::vector<std::string>{"08", "true"}));

  test.handle_response({"$TMSTA", "14", "90", "Hello World", "*73"});
  EXPECT_EQ(test.tmsta_resp_.subcmd_, 90);
  EXPECT_EQ(test.tmsta_resp_.data_, (std::vector<std::string>{"Hello World"}));

  test.handle_response({"$CPERR", "2", "01", "*49"});
  EXPECT_EQ(test.cperr_resp_.err_, tm_robot_listener::ErrorCode::BadArgument);

  test.handle_response({"$CPERR", "2", "02", "*4A"});
  EXPECT_EQ(test.cperr_resp_.err_, tm_robot_listener::ErrorCode::BadCheckSum);

  test.handle_response({"$CPERR", "2", "03", "*4B"});
  EXPECT_EQ(test.cperr_resp_.err_, tm_robot_listener::ErrorCode::BadHeader);

  test.handle_response({"$CPERR", "2", "04", "*4C"});
  EXPECT_EQ(test.cperr_resp_.err_, tm_robot_listener::ErrorCode::InvalidData);

  test.handle_response({"$CPERR", "2", "F1", "*3F"});
  EXPECT_EQ(test.cperr_resp_.err_, tm_robot_listener::ErrorCode::NotInListenNode);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}