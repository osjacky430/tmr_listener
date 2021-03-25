#include <gtest/gtest.h>

#include "tmr_listener/tmr_listener_handle.hpp"

class MsgParseTester final : public tmr_listener::ListenerHandle {
 public:
  tmr_listener::TMSCTResponse tmsct_resp_;
  tmr_listener::TMSTAResponse tmsta_resp_;
  tmr_listener::CPERRResponse cperr_resp_;

 protected:
  tmr_listener::Decision start_task(std::vector<std::string> const& /*unused*/) override {
    return tmr_listener::Decision::Accept;
  }

  tmr_listener::BaseHeaderProductPtr generate_cmd(MessageStatus const /*unused*/) override {
    return tmr_listener::BaseHeaderProductPtr{};
  }

  void response_msg(tmr_listener::TMSCTResponse const& t_resp) override { this->tmsct_resp_ = t_resp; }

  void response_msg(tmr_listener::TMSTAResponse const& t_resp) override { this->tmsta_resp_ = t_resp; }

  void response_msg(tmr_listener::CPERRResponse const& t_resp) override { this->cperr_resp_ = t_resp; }

  using tmr_listener::ListenerHandle::response_msg;
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
  EXPECT_EQ(test.cperr_resp_.err_, tmr_listener::ErrorCode::BadArgument);

  test.handle_response({"$CPERR", "2", "02", "*4A"});
  EXPECT_EQ(test.cperr_resp_.err_, tmr_listener::ErrorCode::BadCheckSum);

  test.handle_response({"$CPERR", "2", "03", "*4B"});
  EXPECT_EQ(test.cperr_resp_.err_, tmr_listener::ErrorCode::BadHeader);

  test.handle_response({"$CPERR", "2", "04", "*4C"});
  EXPECT_EQ(test.cperr_resp_.err_, tmr_listener::ErrorCode::InvalidData);

  test.handle_response({"$CPERR", "2", "F1", "*3F"});
  EXPECT_EQ(test.cperr_resp_.err_, tmr_listener::ErrorCode::NotInListenNode);
}

TEST(SpiritParseTest, ContentMatch) {
  using namespace tmr_listener;
  {
    auto const test_str = "$TMSCT,50,ChangePayload,float payload=0\r\nChangeLoad(payload),*6B";

    auto const p = TMSCT.parse(test_str);

    ASSERT_EQ(p.data_.cmd_.size(), 2);
    EXPECT_EQ(p.length_, 50);
    EXPECT_EQ(p.data_.id_, "ChangePayload");

    auto const var = boost::get<tmr_listener::detail::TMSCTTag::DataFormat::VariableDecl>(&p.data_.cmd_[0]);
    ASSERT_TRUE(var != nullptr);
    EXPECT_EQ(var->type_, "float");
    EXPECT_EQ(var->name_, "payload");
    EXPECT_EQ(var->val_, "0");

    auto const var_2 = boost::get<tmr_listener::detail::TMSCTTag::DataFormat::FunctionCall>(&p.data_.cmd_[1]);
    ASSERT_TRUE(var_2 != nullptr);
    EXPECT_EQ(var_2->name_, "ChangeLoad");
    EXPECT_EQ(var_2->args_, "payload");

    EXPECT_EQ(p.checksum_, "6B");
  }

  {
    auto const test_str =
      "$TMSCT,169,MoveToHome,PTP(\"CPP\",Point[\"Safety_Back\"].Value,100,200,100,true)\r\n"
      "QueueTag(1)\r\n"
      "float[] targetP2 = {90, -35, 125, 0, 90, 0}\r\n"
      "PTP(\"JPP\", targetP2, 100, 200, 100, true)\r\n"
      "QueueTag(2, 1),*6A ";

    auto const p = TMSCT.parse(test_str);

    ASSERT_EQ(p.data_.cmd_.size(), 5);
    EXPECT_EQ(p.length_, 169);
    EXPECT_EQ(p.data_.id_, "MoveToHome");

    auto const var = boost::get<tmr_listener::detail::TMSCTTag::DataFormat::FunctionCall>(&p.data_.cmd_[0]);
    ASSERT_TRUE(var != nullptr);
    EXPECT_EQ(var->name_, "PTP");
    EXPECT_EQ(var->args_, "\"CPP\",Point[\"Safety_Back\"].Value,100,200,100,true");

    auto const var_2 = boost::get<tmr_listener::detail::TMSCTTag::DataFormat::FunctionCall>(&p.data_.cmd_[1]);
    ASSERT_TRUE(var_2 != nullptr);
    EXPECT_EQ(var_2->name_, "QueueTag");
    EXPECT_EQ(var_2->args_, "1");

    auto const var_3 = boost::get<tmr_listener::detail::TMSCTTag::DataFormat::VariableDecl>(&p.data_.cmd_[2]);
    ASSERT_TRUE(var_3 != nullptr);
    EXPECT_EQ(var_3->type_, "float[]");
    EXPECT_EQ(var_3->name_, "targetP2");
    EXPECT_EQ(var_3->val_, "{90, -35, 125, 0, 90, 0}");

    auto const var_4 = boost::get<tmr_listener::detail::TMSCTTag::DataFormat::FunctionCall>(&p.data_.cmd_[3]);
    ASSERT_TRUE(var_4 != nullptr);
    EXPECT_EQ(var_4->name_, "PTP");
    EXPECT_EQ(var_4->args_, "\"JPP\",targetP2,100,200,100,true");

    // ASSERT_TRUE(var_2 != nullptr);
    // EXPECT_EQ(var_2->name_, "ChangeLoad");
    // EXPECT_EQ(var_2->args_, "payload");

    EXPECT_EQ(p.checksum_, "6A");
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}