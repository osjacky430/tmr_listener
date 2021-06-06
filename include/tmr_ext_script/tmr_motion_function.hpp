#ifndef TMR_MOTION_FUNCTION_HPP_
#define TMR_MOTION_FUNCTION_HPP_

#include <boost/algorithm/string_regex.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>
#include <boost/tokenizer.hpp>
#include <functional>
#include <string>
#include <vector>

#include <iostream>

#include "tmr_ext_script/detail/tmr_function.hpp"
#include "tmr_ext_script/detail/tmr_header_tag.hpp"
#include "tmr_prototype/tmr_header.hpp"
#include "tmr_variable.hpp"

namespace tmr_listener {

enum class ErrorCode { NoError, BadArgument, BadCheckSum, BadHeader, InvalidData, NotInListenNode = 0xF1 };

struct Packet {
  std::string header;
  std::size_t length;
  std::vector<std::string> data;
  std::string checksum;
};

struct TMSTAResponse {
  int subcmd_;
  std::vector<std::string> data_{};
};

struct TMSCTResponse {
  std::string id_{""};
  bool script_result_ = false;
  std::vector<int> abnormal_line_{};
};

struct CPERRResponse {
  ErrorCode err_ = ErrorCode::NoError;
};

using MessagePtr = boost::shared_ptr<prototype::MessageBase>;

namespace motion_function {

// clang-format off
#define TMR_MOTION_FUNC(name, ret_type, ...)  constexpr tmr_listener::detail::TMSTCFuncSet<ret_type, __VA_ARGS__> name { #name } 
#define TMR_SUBCMD(name, subcmd, ...)         constexpr tmr_listener::detail::TMSTAFuncSet<__VA_ARGS__> name { #subcmd }
#define TMR_HEADER(name)                      constexpr prototype::Header<tmr_listener::detail::name##Tag> name {}
#define SIGNATURE(...)                        tmr_listener::detail::Function<__VA_ARGS__>
#define RETURN_TYPE(type)                     type
#define TMR_VOID
// clang-format on

/**
 * @brief Motion function FunctionSet instances
 */
TMR_MOTION_FUNC(QueueTag, RETURN_TYPE(bool), SIGNATURE(int), SIGNATURE(int, int));
TMR_MOTION_FUNC(WaitQueueTag, RETURN_TYPE(int), SIGNATURE(int), SIGNATURE(int, int));
TMR_MOTION_FUNC(StopAndClearBuffer, RETURN_TYPE(bool), SIGNATURE(TMR_VOID));
TMR_MOTION_FUNC(Pause, RETURN_TYPE(bool), SIGNATURE(TMR_VOID));
TMR_MOTION_FUNC(Resume, RETURN_TYPE(bool), SIGNATURE(TMR_VOID));
TMR_MOTION_FUNC(PTP, RETURN_TYPE(bool), SIGNATURE(std::string, std::array<float, 6>, int, int, int, bool),
                SIGNATURE(std::string, std::array<float, 6>, int, int, int, bool, std::array<int, 3>),
                SIGNATURE(std::string, float, float, float, float, float, float, int, int, int, bool),
                SIGNATURE(std::string, float, float, float, float, float, float, int, int, int, bool, int, int, int));
TMR_MOTION_FUNC(Line, RETURN_TYPE(bool), SIGNATURE(std::string, std::array<float, 6>, int, int, int, bool),
                SIGNATURE(std::string, float, float, float, float, float, float, int, int, int, bool));
TMR_MOTION_FUNC(Circle, RETURN_TYPE(bool),
                SIGNATURE(std::string, std::array<float, 6>, std::array<float, 6>, int, int, int, bool),
                SIGNATURE(std::string, float, float, float, float, float, float, float, float, float, float, float,
                          float, int, int, int, bool));
TMR_MOTION_FUNC(PLine, RETURN_TYPE(bool), SIGNATURE(std::string, std::array<float, 6>, int, int, int),
                SIGNATURE(std::string, float, float, float, float, float, float, int, int, int));
TMR_MOTION_FUNC(Move_PTP, RETURN_TYPE(bool), SIGNATURE(std::string, std::array<float, 6>, int, int, int, bool),
                SIGNATURE(std::string, float, float, float, float, float, float, int, int, int, bool));
TMR_MOTION_FUNC(Move_Line, RETURN_TYPE(bool), SIGNATURE(std::string, std::array<float, 6>, int, int, int, bool),
                SIGNATURE(std::string, float, float, float, float, float, float, int, int, int, bool));
TMR_MOTION_FUNC(Move_PLine, RETURN_TYPE(bool), SIGNATURE(std::string, std::array<float, 6>, int, int, int),
                SIGNATURE(std::string, float, float, float, float, float, float, int, int, int));
TMR_MOTION_FUNC(ChangeBase, RETURN_TYPE(bool), SIGNATURE(std::string), SIGNATURE(std::array<float, 6>),
                SIGNATURE(float, float, float, float, float, float));
TMR_MOTION_FUNC(ChangeTCP, RETURN_TYPE(bool), SIGNATURE(std::string), SIGNATURE(std::array<float, 6>),
                SIGNATURE(std::array<float, 6>, float), SIGNATURE(std::array<float, 6>, float, std::array<float, 9>),
                SIGNATURE(float, float, float, float, float, float),
                SIGNATURE(float, float, float, float, float, float, float),
                SIGNATURE(float, float, float, float, float, float, float, float, float, float, float, float, float,
                          float, float, float));
TMR_MOTION_FUNC(ChangeLoad, RETURN_TYPE(bool), SIGNATURE(float));

TMR_MOTION_FUNC(PVTEnter, RETURN_TYPE(bool), SIGNATURE(int), SIGNATURE(TMR_VOID));
TMR_MOTION_FUNC(PVTExit, RETURN_TYPE(bool), SIGNATURE(TMR_VOID));
TMR_MOTION_FUNC(PVTPoint, RETURN_TYPE(bool), SIGNATURE(std::array<float, 6>, std::array<float, 6>, float),
                SIGNATURE(float, float, float, float, float, float, float, float, float, float, float, float, float));
TMR_MOTION_FUNC(PVTPause, RETURN_TYPE(bool), SIGNATURE(TMR_VOID));
TMR_MOTION_FUNC(PVTResume, RETURN_TYPE(bool), SIGNATURE(TMR_VOID));

TMR_SUBCMD(InExtScriptCtlMode, 00, SIGNATURE(TMR_VOID));
TMR_SUBCMD(QueueTagDone, 01, SIGNATURE(int));

}  // namespace motion_function

/**
 * @brief TMSCT and TMSTA Header instances
 */
TMR_HEADER(TMSCT);
TMR_HEADER(TMSTA);
TMR_HEADER(CPERR);

/**
 * @brief return empty commmand list
 *
 * @details If we don't want to response to the message immediately, or the data is not ready at that moment, one may
 *          possibily send empty command list to inform TMRobotListener.
 *
 * @return empty command list
 *
 * @code{.cpp}
 *
 *          class SomeListenNodeEventHandler final : public ListenerHandle {
 *           protected:
 *            motion_function::MessagePtr generate_cmd(MessageStatus const t_prev_response) override {
 *              using namespace motion_function;
 *              if (t_prev_response == MessageStatus::Responded) {
 *                // TM robot responded to the command you sent previously
 *                // do something...
 *              }
 *
 *              // TM robot hasn't responded yet
 *              return empty_command_list();  // this line inform TMRobotListener to do other action
 *            }
 *          };
 *
 * @endcode
 */
inline auto empty_command_list() noexcept { return boost::make_shared<prototype::Message<void>>(); }

inline auto dummy_command_list(std::string t_dummy_cmd_id) noexcept {
  return TMSCT << ID{std::move(t_dummy_cmd_id)} << End();
}

using TMSCTPacket = tmr_listener::prototype::Header<tmr_listener::detail::TMSCTTag>::Packet;
using TMSTAPacket = tmr_listener::prototype::Header<tmr_listener::detail::TMSTATag>::Packet;

}  // namespace tmr_listener

BOOST_FUSION_ADAPT_STRUCT(tmr_listener::TMSCTPacket, length_, data_, checksum_)
BOOST_FUSION_ADAPT_STRUCT(tmr_listener::TMSTAPacket, length_, data_, checksum_)

#endif