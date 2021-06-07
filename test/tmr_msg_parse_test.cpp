#include <gtest/gtest.h>

#include "tmr_ethernet/tmr_eth_slave.hpp"
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

  tmr_listener::MessagePtr generate_cmd(MessageStatus const /*unused*/) override { return tmr_listener::MessagePtr{}; }

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

TEST(SpiritParseTest, TMSCTContentMatch) {
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

    EXPECT_EQ(p.checksum_, "6A");
  }
}

TEST(SpiritParseTest, TMSVRContentMatch) {
  using namespace tmr_listener;
  using namespace std::string_literals;

  {
    constexpr auto input =
      R"($TMSVR,1009,1,3,[{"Item":"Robot_Link","Value":1},)"
      R"({"Item":"Robot_Error","Value":false},)"
      R"({"Item":"Project_Run","Value":true},)"
      R"({"Item":"Project_Pause","Value":false},)"
      R"({"Item":"Safeguard_A","Value":false},)"
      R"({"Item":"ESTOP","Value":false},)"
      R"({"Item":"Camera_Light","Value":0},)"
      R"({"Item":"Error_Code","Value":0},)"
      R"({"Item":"Joint_Angle","Value":[89.4597,-35.00033,125.000435,-0.000126898289,90.0,0.0005951971]},)"
      R"({"Item":"Coord_Base_Tool","Value":[158.595123,252.253464,490.568024,-179.6098,-0.190238521,179.489044]},)"
      R"({"Item":"TCP_Force","Value":[-1.29178234E-05,8.121608E-06,0.0]},)"
      R"({"Item":"TCP_Force3D","Value":1.52587891E-05},)"
      R"({"Item":"TCP_Speed","Value":[5.605194E-45,-5.605194E-45,-5.605194E-45,-5.18032E-41,-1.055388E-40,-1.84061226E-38]},)"
      R"({"Item":"TCP_Speed3D","Value":0.0},)"
      R"({"Item":"Joint_Speed","Value":[0.000297520659,0.0,0.00035643563,0.0,0.0,0.000445544545]},)"
      R"({"Item":"Joint_Torque","Value":[-8997.56,8997.56,-32431.0977,-6253.28,827.64,3799.62]},)"
      R"({"Item":"Project_Speed","Value":60},)"
      R"({"Item":"MA_Mode","Value":1},)"
      R"({"Item":"rng","Value":0}],*0E)";

    auto const packet = TMSVR.parse(input);
    auto const parsed = packet.data_.parse_raw_content(packet.data_.raw_content_);

    EXPECT_EQ(packet.length_, 1009);
    EXPECT_EQ(packet.data_.id_, "1");
    EXPECT_EQ(packet.data_.mode_, Mode::Json);
    EXPECT_EQ(parsed.size(), 19);
    EXPECT_EQ(packet.checksum_, "0E");
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}