#include <gtest/gtest.h>

#include "tmr_ethernet/tmr_eth_slave.hpp"
#include "tmr_listener/tmr_listener_handle.hpp"

TEST(SpiritParse, TMSCTClientRequestTest) {
  using namespace tmr_listener;
  {
    constexpr auto test_str = "$TMSCT,50,ChangePayload,float payload=0\r\nChangeLoad(payload),*6B";

    auto const p = TMSCTHeader::parse(test_str);

    auto const client_request = boost::get<detail::TMSCTTag::DataFormat::ClientRequest>(&p.data_.cmd_);
    ASSERT_EQ(client_request->size(), 2);
    EXPECT_EQ(p.length_, 50);
    EXPECT_EQ(p.data_.id_, "ChangePayload");

    auto const var = boost::get<detail::TMSCTTag::DataFormat::VariableDecl>(&client_request->at(0));
    ASSERT_TRUE(var != nullptr);
    EXPECT_EQ(var->type_, "float");
    EXPECT_EQ(var->name_, "payload");
    EXPECT_EQ(var->val_, "0");

    auto const var_2 = boost::get<detail::TMSCTTag::DataFormat::FunctionCall>(&client_request->at(1));
    ASSERT_TRUE(var_2 != nullptr);
    EXPECT_EQ(var_2->name_, "ChangeLoad");
    EXPECT_EQ(var_2->args_, "payload");

    EXPECT_EQ(p.checksum_, "6B");
  }

  {
    constexpr auto test_str =
      "$TMSCT,169,MoveToHome,PTP(\"CPP\",Point[\"Safety_Back\"].Value,100,200,100,true)\r\n"
      "QueueTag(1)\r\n"
      "float[] targetP2 = {90, -35, 125, 0, 90, 0}\r\n"
      "PTP(\"JPP\", targetP2, 100, 200, 100, true)\r\n"
      "QueueTag(2, 1),*6A ";

    auto const p = TMSCTHeader::parse(test_str);

    auto const client_request = boost::get<detail::TMSCTTag::DataFormat::ClientRequest>(&p.data_.cmd_);
    ASSERT_TRUE(client_request != nullptr);
    ASSERT_EQ(client_request->size(), 5);
    EXPECT_EQ(p.length_, 169);
    EXPECT_EQ(p.data_.id_, "MoveToHome");

    auto const var = boost::get<detail::TMSCTTag::DataFormat::FunctionCall>(&client_request->at(0));
    ASSERT_TRUE(var != nullptr);
    EXPECT_EQ(var->name_, "PTP");
    EXPECT_EQ(var->args_, R"("CPP",Point["Safety_Back"].Value,100,200,100,true)");

    auto const var_2 = boost::get<detail::TMSCTTag::DataFormat::FunctionCall>(&client_request->at(1));
    ASSERT_TRUE(var_2 != nullptr);
    EXPECT_EQ(var_2->name_, "QueueTag");
    EXPECT_EQ(var_2->args_, "1");

    auto const var_3 = boost::get<detail::TMSCTTag::DataFormat::VariableDecl>(&client_request->at(2));
    ASSERT_TRUE(var_3 != nullptr);
    EXPECT_EQ(var_3->type_, "float[]");
    EXPECT_EQ(var_3->name_, "targetP2");
    EXPECT_EQ(var_3->val_, "{90, -35, 125, 0, 90, 0}");

    auto const var_4 = boost::get<detail::TMSCTTag::DataFormat::FunctionCall>(&client_request->at(3));
    ASSERT_TRUE(var_4 != nullptr);
    EXPECT_EQ(var_4->name_, "PTP");
    EXPECT_EQ(var_4->args_, R"("JPP",targetP2,100,200,100,true)");

    EXPECT_EQ(p.checksum_, "6A");
  }
}

TEST(SpiritParse, TMSCTServerResponseTest) {
  using namespace tmr_listener;
  {
    constexpr auto response = "$TMSCT,9,4,ERROR;1,*02\r\n";

    auto const parsed = TMSCTHeader::parse(response);
    auto const cmd    = boost::get<detail::TMSCTTag::DataFormat::ServerResponse>(&parsed.data_.cmd_);
    ASSERT_TRUE(cmd != nullptr);

    auto const script_result = boost::get<detail::TMSCTTag::DataFormat::ScriptResult>(cmd);
    ASSERT_TRUE(script_result != nullptr);
    EXPECT_EQ(script_result->abnormal_lines_.size(), 1);
    EXPECT_EQ(script_result->abnormal_lines_.at(0), 1);
  }

  {
    constexpr auto response = "$TMSCT,4,1,OK,*5C\r\n";

    auto const parsed = TMSCTHeader::parse(response);
    auto const cmd    = boost::get<detail::TMSCTTag::DataFormat::ServerResponse>(&parsed.data_.cmd_);
    ASSERT_TRUE(cmd != nullptr);

    auto const script_result = boost::get<detail::TMSCTTag::DataFormat::ScriptResult>(cmd);
    ASSERT_TRUE(script_result != nullptr);
    EXPECT_TRUE(script_result->abnormal_lines_.empty());
  }

  {
    constexpr auto response = "$TMSCT,4,1,OK;1;2,*5C\r\n";

    auto const parsed = TMSCTHeader::parse(response);
    auto const cmd    = boost::get<detail::TMSCTTag::DataFormat::ServerResponse>(&parsed.data_.cmd_);
    ASSERT_TRUE(cmd != nullptr);

    auto const script_result = boost::get<detail::TMSCTTag::DataFormat::ScriptResult>(cmd);
    ASSERT_TRUE(script_result != nullptr);
    EXPECT_EQ(script_result->abnormal_lines_.size(), 2);
    EXPECT_EQ(script_result->abnormal_lines_.at(0), 1);
    EXPECT_EQ(script_result->abnormal_lines_.at(1), 2);
  }
}

TEST(SpiritParse, CPERRResponseMatch) {
  using namespace tmr_listener;

  {
    constexpr auto response = "$CPERR,2,01,*49\r\n";

    auto const parsed = CPERRHeader::parse(response);
    EXPECT_EQ(parsed.data_.err_, ErrorCode::BadArgument);
  }

  {
    constexpr auto response = "$CPERR,2,02,*4A\r\n";

    auto const parsed = CPERRHeader::parse(response);
    EXPECT_EQ(parsed.data_.err_, ErrorCode::BadCheckSum);
  }

  {
    constexpr auto response = "$CPERR,2,03,*4B\r\n";

    auto const parsed = CPERRHeader::parse(response);
    EXPECT_EQ(parsed.data_.err_, ErrorCode::BadHeader);
  }

  {
    constexpr auto response = "$CPERR,2,04,*4C\r\n";

    auto const parsed = CPERRHeader::parse(response);
    EXPECT_EQ(parsed.data_.err_, ErrorCode::InvalidData);
  }

  {
    constexpr auto response = "$CPERR,2,F1,*3F\r\n";

    auto const parsed = CPERRHeader::parse(response);
    EXPECT_EQ(parsed.data_.err_, ErrorCode::NotInListenNode);
  }
}

TEST(SpiritParse, TMSVRContentMatch) {
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

    auto const packet = TMSVRHeader::parse(input);
    auto const parsed = TMSVRPacket::DataFrame::parse_raw_content(packet.data_.raw_content_);

    EXPECT_EQ(packet.length_, 1009);
    EXPECT_EQ(packet.data_.id_, "1");
    EXPECT_EQ(packet.data_.mode_, Mode::Json);
    EXPECT_EQ(parsed.size(), 19);

    EXPECT_EQ(parse_as<int>(parsed.at(0).value_), 1);   // Robot_Link
    EXPECT_FALSE(parse_as<bool>(parsed.at(1).value_));  // Robot_Error
    EXPECT_TRUE(parse_as<bool>(parsed.at(2).value_));   // Project_Run
    EXPECT_FALSE(parse_as<bool>(parsed.at(3).value_));  // Project_Pause
    EXPECT_FALSE(parse_as<bool>(parsed.at(4).value_));  // Safeguard_A
    EXPECT_FALSE(parse_as<bool>(parsed.at(5).value_));  // ESTOP
    EXPECT_EQ(parse_as<int>(parsed.at(6).value_), 0);   // Camera_Light
    EXPECT_EQ(parse_as<int>(parsed.at(7).value_), 0);   // Error_Code

    // actually, I'm quite surprise these double precision floating point comparisons will pass
    EXPECT_EQ((parse_as<double, 6>(parsed.at(8).value_)),  // Joint_Angle
              (std::array<double, 6>{89.4597, -35.00033, 125.000435, -0.000126898289, 90.0, 0.0005951971}));

    EXPECT_EQ((parse_as<double, 6>(parsed.at(9).value_)),  // Cood_Base_Tool
              (std::array<double, 6>{158.595123, 252.253464, 490.568024, -179.6098, -0.190238521, 179.489044}));
    EXPECT_THROW((parse_as<double, 5>(parsed.at(9).value_)), std::out_of_range);
    EXPECT_THROW((parse_as<double, 7>(parsed.at(9).value_)), std::invalid_argument);

    EXPECT_EQ((parse_as<double, 3>(parsed.at(10).value_)),  // TCP_Force
              (std::array<double, 3>{-1.29178234E-05, 8.121608E-06, 0.0}));
    EXPECT_EQ((parse_as<double>(parsed.at(11).value_)), 1.52587891E-05);  // TCP_Force3D
    EXPECT_EQ((parse_as<double, 6>(parsed.at(12).value_)),                // TCP_Speed
              (std::array<double, 6>{5.605194E-45, -5.605194E-45, -5.605194E-45, -5.18032E-41, -1.055388E-40,
                                     -1.84061226E-38}));
    EXPECT_EQ(parse_as<double>(parsed.at(13).value_), 0.0);  // TCP_Speed3D
    EXPECT_EQ((parse_as<double, 6>(parsed.at(14).value_)),   // Joint_Speed
              (std::array<double, 6>{0.000297520659, 0.0, 0.00035643563, 0.0, 0.0, 0.000445544545}));
    EXPECT_EQ((parse_as<double, 6>(parsed.at(15).value_)),  // Joint_Torque
              (std::array<double, 6>{-8997.56, 8997.56, -32431.0977, -6253.28, 827.64, 3799.62}));

    EXPECT_EQ(parse_as<int>(parsed.at(16).value_), 60);  // Project_Speed
    EXPECT_EQ(parse_as<int>(parsed.at(17).value_), 1);   // MA_Mode
    EXPECT_EQ(parse_as<int>(parsed.at(18).value_), 0);   // rng

    EXPECT_EQ(packet.checksum_, "0E");
  }
}
