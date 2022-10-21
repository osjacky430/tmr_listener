#include <gtest/gtest.h>

#include "tmr_ethernet/tmr_eth_slave.hpp"
#include "tmr_listener/tmr_listener.hpp"
#include "tmr_listener/tmr_listener_handle.hpp"

TEST(SpiritParse, TMSCTServerResponseTest) {
  using namespace tmr_listener;
  {
    constexpr auto response = "$TMSCT,9,4,ERROR;1,*02\r\n";

    auto const parsed               = TMSCTHeader::parse(response);
    auto const* const script_result = boost::get<TMSCTResponse>(&parsed.data_.resp_);
    ASSERT_TRUE(script_result != nullptr);
    EXPECT_EQ(script_result->abnormal_lines_.size(), 1);
    EXPECT_EQ(script_result->abnormal_lines_.at(0), 1);
  }

  {
    constexpr auto response = "$TMSCT,4,1,OK,*5C\r\n";

    auto const parsed               = TMSCTHeader::parse(response);
    auto const* const script_result = boost::get<TMSCTResponse>(&parsed.data_.resp_);
    ASSERT_TRUE(script_result != nullptr);
    EXPECT_TRUE(script_result->abnormal_lines_.empty());
  }

  {
    constexpr auto response = "$TMSCT,4,1,OK;1;2,*5C\r\n";

    auto const parsed               = TMSCTHeader::parse(response);
    auto const* const script_result = boost::get<TMSCTResponse>(&parsed.data_.resp_);

    ASSERT_TRUE(script_result != nullptr);
    EXPECT_EQ(script_result->abnormal_lines_.size(), 2);
    EXPECT_EQ(script_result->abnormal_lines_.at(0), 1);
    EXPECT_EQ(script_result->abnormal_lines_.at(1), 2);
  }
}

TEST(SpiritParse, TMSTAResponseMatch) {
  using namespace tmr_listener;
  using namespace std::string_literals;

  {
    constexpr auto response = "$TMSTA,29,00,true,UltrasonicFail,-12131,*5B\r\n";

    auto const parsed     = TMSTAHeader::parse(response);
    auto const* const cmd = boost::get<TMSTAResponse::Subcmd00>(&parsed.data_.resp_);
    ASSERT_TRUE(cmd != nullptr);

    EXPECT_TRUE(cmd->entered_);
    EXPECT_EQ(cmd->node_name_, "UltrasonicFail,-12131"s);
  }

  {
    constexpr auto response = "$TMSTA,10,01,08,true,*6D\r\n";
    auto const parsed       = TMSTAHeader::parse(response);
    auto const* const cmd   = boost::get<TMSTAResponse::Subcmd01>(&parsed.data_.resp_);
    ASSERT_TRUE(cmd != nullptr);

    EXPECT_EQ(cmd->tag_number_, "08");
    EXPECT_EQ(cmd->tag_stat_, TagNumberStatus::Complete);
  }

  {
    constexpr auto response = "$TMSTA,14,90,Hello World,*73\r\n";
    auto const parsed       = TMSTAHeader::parse(response);
    auto const* const cmd   = boost::get<TMSTAResponse::DataMsg>(&parsed.data_.resp_);
    ASSERT_TRUE(cmd != nullptr);

    EXPECT_EQ(cmd->cmd_, 90);
    EXPECT_EQ(cmd->data_, "Hello World"s);
  }

  {
    constexpr auto response = "$TMSTA,10,91,123.456,*7E\r\n";
    auto const parsed       = TMSTAHeader::parse(response);
    auto const* const cmd   = boost::get<TMSTAResponse::DataMsg>(&parsed.data_.resp_);
    ASSERT_TRUE(cmd != nullptr);

    EXPECT_EQ(cmd->cmd_, 91);
    EXPECT_EQ(cmd->data_, "123.456"s);
  }

  {
    constexpr auto response = "$TMSTA,67,90,{-248.7316,-154.6399,-130.5106,-0.04263335,-0.6727331,0.8938164},*52\r\n";
    auto const parsed       = TMSTAHeader::parse(response);
    auto const* const cmd   = boost::get<TMSTAResponse::DataMsg>(&parsed.data_.resp_);
    ASSERT_TRUE(cmd != nullptr);

    EXPECT_EQ(cmd->cmd_, 90);
    EXPECT_EQ(cmd->data_, "{-248.7316,-154.6399,-130.5106,-0.04263335,-0.6727331,0.8938164}"s);
  }
}

TEST(SpiritParse, CPERRResponseMatch) {
  using namespace tmr_listener;
  using namespace std::string_literals;

  auto const test_cperr_msg_equal = [](auto const& t_string, auto const& t_err_code) {
    auto const parsed = CPERRHeader::parse(t_string);
    EXPECT_EQ(parsed.data_.resp_.err_, t_err_code);
  };

  test_cperr_msg_equal("$CPERR,2,01,*49\r\n"s, ErrorCode::BadArgument);
  test_cperr_msg_equal("$CPERR,2,02,*4A\r\n"s, ErrorCode::BadCheckSum);
  test_cperr_msg_equal("$CPERR,2,03,*4B\r\n"s, ErrorCode::BadHeader);
  test_cperr_msg_equal("$CPERR,2,04,*4C\r\n"s, ErrorCode::InvalidData);
  test_cperr_msg_equal("$CPERR,2,F1,*3F\r\n"s, ErrorCode::NotInListenNode);
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

    // actually, I'm quite surprised that these double precision floating point comparisons will pass
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
