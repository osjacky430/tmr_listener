#include <gtest/gtest.h>

#include "tmr_ethernet/tmr_eth_rw.hpp"
#include "tmr_ext_script/tmr_motion_function.hpp"
#include "tmr_ext_script/tmr_parameterized_object.hpp"

using namespace tmr_listener;
using namespace motion_function;
using namespace prototype;
using namespace std::string_literals;

TEST(ChecksumTest, ChecksumStringMatch) {
  EXPECT_EQ(calculate_checksum("$TMSTA,10,01,08,true,"), "6D");
  EXPECT_EQ(calculate_checksum("$TMSTA,5,01,15,"), "6F");
  EXPECT_EQ(calculate_checksum("$TMSCT,5,10,OK,"), "6D");
  EXPECT_EQ(calculate_checksum("$TMSCT,4,1,OK,"), "5C");
  EXPECT_EQ(calculate_checksum("$TMSCT,8,2,OK;2;3,"), "52");
  EXPECT_EQ(calculate_checksum("$TMSCT,13,3,ERROR;1;2;3,"), "3F");
  EXPECT_EQ(calculate_checksum("$TMSCT,25,1,ChangeBase(\"RobotBase\"),"), "08");
  EXPECT_EQ(calculate_checksum("$TMSCT,64,2,ChangeBase(\"RobotBase\")\r\nChangeTCP(\"NOTOOL\")\r\nChangeLoad(10.1),"),
            "68");
  EXPECT_EQ(calculate_checksum("$TMSCT,40,3,int var_i = 100\r\nvar_i = 1000\r\nvar_i++,"), "5A");
  EXPECT_EQ(calculate_checksum("$TMSCT,42,4,int var_i = 100\r\nvar_i = 1000\r\nvar_i++\r\n,"), "58");
  EXPECT_EQ(calculate_checksum("$TMSCT,9,4,ERROR;1,"), "02");
  EXPECT_EQ(calculate_checksum("$TMSTA,9,00,false,,"), "37");
  EXPECT_EQ(calculate_checksum("$TMSTA,15,00,true,Listen1,"), "79");
  EXPECT_EQ(calculate_checksum("$TMSTA,2,00,"), "41");
  EXPECT_EQ(calculate_checksum("$TMSTA,10,01,15,none,"), "7D");
  EXPECT_EQ(calculate_checksum("$TMSTA,5,01,88,"), "6B");
  EXPECT_EQ(calculate_checksum("$TMSTA,10,01,88,none,"), "79");
  EXPECT_EQ(calculate_checksum("$TMSCT,-100,1,ChangeBase(\"RobotBase\"),"), "13");
  EXPECT_EQ(calculate_checksum("$CPERR,2,01,"), "49");
  EXPECT_EQ(calculate_checksum("$CPERR,2,02,"), "4A");
  EXPECT_EQ(calculate_checksum("$TMsct,25,1,ChangeBase(\"RobotBase\"),"), "28");
  EXPECT_EQ(calculate_checksum("$CPERR,2,03,"), "4B");
  EXPECT_EQ(calculate_checksum("$TMSCT,23,ChangeBase(\"RobotBase\"),"), "13");
  EXPECT_EQ(calculate_checksum("$CPERR,2,04,"), "4C");
  EXPECT_EQ(calculate_checksum("$TMSTA,4,XXXX,"), "47");
  EXPECT_EQ(calculate_checksum("$CPERR,2,F1,"), "3F");

  EXPECT_EQ(calculate_checksum("$TMSCT,172,2,float[] targetP1= {0,0,90,0,90,0}\r\n"
                               "PTP(”JPP”,targetP1,10,200,0,false)\r\nQueueTag(1)\r\n"
                               "float[] targetP2 = { 0, 90, 0, 90, 0, 0 }\r\n"
                               "PTP(”JPP”, targetP2, 10, 200, 10, false)\r\n"
                               "QueueTag(2)\r\n,"),
            "49");
}

#define EXPECT(TYPE) Expression<TYPE>
#define VARIABLE_BINARY_OP_TEST(VAR_1, OP, VAR_2, RESULT_TYPE)                                        \
  {                                                                                                   \
    auto expr = VAR_1 OP VAR_2;                                                                       \
    static_assert(std::is_same<decltype(expr), RESULT_TYPE>::value, "Expression type doesn't match"); \
    EXPECT_EQ(expr.to_str(), "(" #VAR_1 #OP #VAR_2 ")");                                              \
  }

TEST(VariableTest, BinaryOperator) {
  Variable<int> other_int{"other_int"};
  Variable<int> int_var{"int_var"};
  Variable<float> float_var{"float_var"};

  Variable<std::array<float, 6>> some_point{"some_point"};
  Variable<std::array<float, 6>> other_point{"other_point"};

  EXPECT_EQ(declare(int_var, 0).to_str(), "int int_var=0");
  EXPECT_EQ(declare(other_int, int_var).to_str(), "int other_int=int_var");
  EXPECT_EQ(declare(some_point, other_point).to_str(), "float[] some_point=other_point");

  VARIABLE_BINARY_OP_TEST(int_var, =, 1, EXPECT(int));
  VARIABLE_BINARY_OP_TEST(int_var, =, float_var, EXPECT(int));

  VARIABLE_BINARY_OP_TEST(int_var, +, 1, EXPECT(int));
  VARIABLE_BINARY_OP_TEST(1, +, int_var, EXPECT(int));

  VARIABLE_BINARY_OP_TEST(1.5, +, int_var, EXPECT(double));
  VARIABLE_BINARY_OP_TEST(int_var, +, 1.5, EXPECT(double));

  VARIABLE_BINARY_OP_TEST(int_var, +, other_int, EXPECT(int));

  VARIABLE_BINARY_OP_TEST(int_var, +, float_var, EXPECT(float));
}

TEST(VariableTest, UnaryOperator) {
  Variable<int> int_var{"int_var"};
  Variable<bool> bool_var{"bool_var"};

  {
    auto expr = int_var++;
    static_assert(std::is_same<decltype(expr), Expression<int>>::value, "Expression type doesn't match");
    EXPECT_EQ(expr.to_str(), "(int_var++)");
  }

  {
    auto expr = ++int_var;
    static_assert(std::is_same<decltype(expr), Expression<int>>::value, "Expression type doesn't match");
    EXPECT_EQ(expr.to_str(), "(++int_var)");
  }

  {
    auto expr = !bool_var;
    static_assert(std::is_same<decltype(expr), Expression<bool>>::value, "Expression type doesn't match");
    EXPECT_EQ(expr.to_str(), "(!bool_var)");
  }

  {
    auto expr = ~int_var;
    static_assert(std::is_same<decltype(expr), Expression<int>>::value, "Expression type doesn't match");
    EXPECT_EQ(expr.to_str(), "(~int_var)");
  }

  {
    auto expr = -int_var;
    static_assert(std::is_same<decltype(expr), Expression<int>>::value, "Expression type doesn't match");
    EXPECT_EQ(expr.to_str(), "(-int_var)");
  }

  {
    auto expr = +int_var;
    static_assert(std::is_same<decltype(expr), Expression<int>>::value, "Expression type doesn't match");
    EXPECT_EQ(expr.to_str(), "(+int_var)");
  }
}

TEST(ExpressionTest, BinaryOperator) {
  Variable<int> int_var{"int_var"};
  Variable<int> other_int{"other_int"};
  Variable<float> float_var{"float_var"};

  Expression<int> int_expr       = int_var + other_int;
  Expression<int> other_int_expr = int_var + 1;
  Expression<float> float_expr   = int_var + float_var;

  {
    auto add_two_int_expr = int_expr + other_int_expr;
    static_assert(std::is_same<decltype(add_two_int_expr), Expression<int>>::value, "Expression type doesn't match");
    EXPECT_EQ(add_two_int_expr.to_str(), "((int_var+other_int)+(int_var+1))");
  }

  {  // expression with different type
    auto add_int_to_float_expr = int_expr + float_expr;
    static_assert(std::is_same<decltype(add_int_to_float_expr), Expression<float>>::value,
                  "Expression type doesn't match");
    EXPECT_EQ(add_int_to_float_expr.to_str(), "((int_var+other_int)+(int_var+float_var))");
  }

  {  // expression + r-value
    auto add_int_expr = int_expr + 1;
    static_assert(std::is_same<decltype(add_int_expr), Expression<int>>::value, "Expression type doesn't match");
    EXPECT_EQ(add_int_expr.to_str(), "((int_var+other_int)+1)");
  }

  {  // expression + variable
    auto add_int_expr = int_expr + int_var;
    static_assert(std::is_same<decltype(add_int_expr), Expression<int>>::value, "Expression type doesn't match");
    EXPECT_EQ(add_int_expr.to_str(), "((int_var+other_int)+int_var)");
  }

  {  // variable + expression
    auto add_int_expr = int_var + int_expr;
    static_assert(std::is_same<decltype(add_int_expr), Expression<int>>::value, "Expression type doesn't match");
    EXPECT_EQ(add_int_expr.to_str(), "(int_var+(int_var+other_int))");
  }

  {
    Variable<bool> bool_var{"bool_var"};

    auto tern_expr_3_var = ternary_expr<int>(bool_var, int_var, float_var);
    static_assert(std::is_same<decltype(tern_expr_3_var), Expression<int>>::value, "Expression type doesn't match");
    EXPECT_EQ(tern_expr_3_var.to_str(), "(bool_var?int_var:float_var)");

    auto tern_expr_3_expr = ternary_expr<int>(int_var == 1, int_var + other_int, float_var + int_var);
    static_assert(std::is_same<decltype(tern_expr_3_expr), Expression<int>>::value, "Expression type doesn't match");
    EXPECT_EQ(tern_expr_3_expr.to_str(), "((int_var==1)?(int_var+other_int):(float_var+int_var))");
  }
}

TEST(TMMsgGen, CommandAsExpression) {
  // the point here is to make sure the type of expression for the motion function is correct
  auto const change_base_command = ChangeBase("RobotBase"s);
  static_assert(std::is_same<decltype(change_base_command.as_expression()), Expression<bool>>::value, "Type not match");
  EXPECT_EQ(change_base_command.as_expression().to_str(), R"(ChangeBase("RobotBase"))"s);

  auto const wait_queue_tag = WaitQueueTag(1);
  static_assert(std::is_same<decltype(wait_queue_tag.as_expression()), Expression<int>>::value, "Type not match");
  EXPECT_EQ(wait_queue_tag.as_expression().to_str(), "WaitQueueTag(1)"s);
}

TEST(TMMsgGen, InvalidIDWillThrowAtRuntime) {
  EXPECT_FALSE(ID::rule_satisfied("Su%NeNe7"));
  EXPECT_FALSE(ID::rule_satisfied("Super_NeNe7"));
  EXPECT_FALSE(ID::rule_satisfied("SuperNeNe_"));

  EXPECT_THROW(ID{"Su%NeNe7"}, std::invalid_argument);
  EXPECT_THROW(ID{"Super_NeNe7"}, std::invalid_argument);
  EXPECT_THROW(ID{"SuperNeNe_"}, std::invalid_argument);
  EXPECT_THROW(ID{"Su%NeNe7"s}, std::invalid_argument);
  EXPECT_THROW(ID{"Super_NeNe7"s}, std::invalid_argument);
  EXPECT_THROW(ID{"SuperNeNe_"s}, std::invalid_argument);
}

TEST(TMMsgGen, TMSCTStringMatch) {
  {
    auto const command = TMSCT << ID{"1"} << ChangeBase("RobotBase"s) << End();
    EXPECT_EQ(command->to_str(), "$TMSCT,25,1,ChangeBase(\"RobotBase\"),*08\r\n");
  }

  {
    auto const command = TMSTA << QueueTagDone(88) << End();
    EXPECT_EQ(command->to_str(), "$TMSTA,5,01,88,*6B\r\n");
  }

  {  // building command in single step
    Variable<std::array<float, 6>> targetP1{"targetP1"};
    Variable<std::array<float, 6>> targetP2{"targetP2"};

    auto const command = TMSCT << ID{"2"} << declare(targetP1, std::array<float, 6>{205, -35, 125, 0, 90, 0})
                               << PTP("JPP"s, targetP1, 10, 200, 0, false) << QueueTag(1)
                               << declare(targetP2, std::array<float, 6>{90, -35, 125, 0, 90, 0})
                               << PTP("JPP"s, targetP2, 10, 200, 10, false) << QueueTag(2) << End();
    EXPECT_EQ(command->to_str(),
              "$TMSCT,176,2,float[] targetP1={205,-35,125,0,90,0}\r\n"
              "PTP(\"JPP\",targetP1,10,200,0,false)\r\n"
              "QueueTag(1)\r\n"
              "float[] targetP2={90,-35,125,0,90,0}\r\n"
              "PTP(\"JPP\",targetP2,10,200,10,false)\r\n"
              "QueueTag(2),*54\r\n");
  }

  {  // building command multistepped
    Variable<std::array<float, 6>> targetP1{"targetP1"};
    Variable<std::array<float, 6>> targetP2{"targetP2"};

    auto command = TMSCT << ID{"2"} << declare(targetP1, std::array<float, 6>{205, -35, 125, 0, 90, 0});
    command << PTP("JPP"s, targetP1, 10, 200, 0, false) << QueueTag(1);
    command << declare(targetP2, std::array<float, 6>{90, -35, 125, 0, 90, 0});
    command << PTP("JPP"s, targetP2, 10, 200, 10, false) << QueueTag(2);
    auto const result = command << End();
    EXPECT_EQ(result->to_str(),
              "$TMSCT,176,2,float[] targetP1={205,-35,125,0,90,0}\r\n"
              "PTP(\"JPP\",targetP1,10,200,0,false)\r\n"
              "QueueTag(1)\r\n"
              "float[] targetP2={90,-35,125,0,90,0}\r\n"
              "PTP(\"JPP\",targetP2,10,200,10,false)\r\n"
              "QueueTag(2),*54\r\n");
  }
}

TEST(TMMsgGen, TMSTAMsgStringMatch) {
  {
    auto const command = TMSTA << InExtScriptCtlMode() << End();
    EXPECT_EQ(command->to_str(), "$TMSTA,2,00,*41\r\n");
  }

  {
    auto const command = TMSTA << QueueTagDone(15) << End();
    EXPECT_EQ(command->to_str(), "$TMSTA,5,01,15,*6F\r\n");
  }
}

TEST(TMMsgGen, TMSVRMsgStringMatch) {
  {
    auto const read_req = generate_read_req("TCP_Mass"s);
    EXPECT_EQ(read_req.item_, R"("TCP_Mass")"s);
    EXPECT_EQ(read_req.to_str(), R"({"Item":"TCP_Mass"})"s);

    auto const to_read = TMSVR << ID{"Q3"} << read_req << End();
    EXPECT_EQ(to_read->to_str(),
              "$TMSVR,27,Q3,13,"
              R"([{"Item":"TCP_Mass"}])"
              ",*3C\r\n"s);

    auto const write_req = generate_write_req("Ctrl_DO0"s, "1"s);
    EXPECT_EQ(write_req.item_, R"("Ctrl_DO0")"s);
    EXPECT_EQ(write_req.value_, "1"s);
    EXPECT_EQ(write_req.to_str(), R"({"Item":"Ctrl_DO0","Value":1})"s);

    auto const write_req_1 = generate_write_req("Ctrl_DO1"s, "0");
    auto const write_req_2 = generate_write_req("g_ss", R"(["Hello","TM","Robot"])");
    auto const to_write    = TMSVR << ID{"T9"} << write_req << write_req_1 << write_req_2 << End();
    EXPECT_EQ(to_write->to_str(), R"($TMSVR,113,T9,3,[{"Item":"Ctrl_DO0","Value":1},{"Item":"Ctrl_DO1","Value":0},)"
                                  R"({"Item":"g_ss","Value":["Hello","TM","Robot"]}],*7C)"
                                  "\r\n"s);
  }

  {
    auto const read_req = generate_read_req(R"("Stick_PlayPause")"s);
    EXPECT_EQ(read_req.item_, R"("Stick_PlayPause")"s);
    EXPECT_EQ(read_req.to_str(), R"({"Item":"Stick_PlayPause"})"s);

    auto const write_req = generate_write_req(R"("Stick_PlayPause")"s, "true"s);
    EXPECT_EQ(write_req.item_, R"("Stick_PlayPause")"s);
    EXPECT_EQ(write_req.value_, "true"s);
    EXPECT_EQ(write_req.to_str(), R"({"Item":"Stick_PlayPause","Value":true})"s);
  }
}