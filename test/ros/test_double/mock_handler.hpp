#ifndef MOCK_HANDLER_HPP_
#define MOCK_HANDLER_HPP_

#include <gmock/gmock.h>
#include <tuple>
#include <utility>

#include "../tmr_test_constant.hpp"
#include "tmr_listener/tmr_listener.hpp"
#include "tmr_listener/tmr_listener_handle.hpp"

namespace tmr_listener {

template <std::size_t Idx>
class HandlerMock : public ListenerHandle {
 public:
  using ListenerHandle::response_msg;

  MOCK_METHOD(MessagePtr, generate_cmd, (MessageStatus const t_msg_status), (override));
  MOCK_METHOD(Decision, start_task, (std::string const &), (override));
  MOCK_METHOD(void, response_msg, (TMSTAResponse::Subcmd00 const &), (override));
  MOCK_METHOD(void, response_msg, (TMSTAResponse::Subcmd01 const &), (override));
  MOCK_METHOD(void, response_msg, (TMSCTResponse const &), (override));
  MOCK_METHOD(void, response_msg, (CPERRResponse const &), (override));
};

template <std::size_t Idx>
using HandlerMockPtr = boost::shared_ptr<HandlerMock<Idx>>;

template <typename T, std::size_t N>
struct have_handler_mock_n_expectation_setter {
  template <typename C, typename = decltype(std::declval<C>().set_expectation(boost::shared_ptr<HandlerMock<N>>()))>
  static std::true_type match_setter_signature(int) noexcept;

  template <typename C>
  static std::false_type match_setter_signature(...) noexcept;

  static constexpr bool value = decltype(match_setter_signature<T>(0))::value;
};

template <typename T, std::size_t n = 0, typename = void>
struct handler_mock_counter;

template <typename T, std::size_t N>
struct handler_mock_counter<T, N, std::enable_if_t<have_handler_mock_n_expectation_setter<T, N>::value>> {
  static constexpr auto value = handler_mock_counter<T, N + 1>::value;
};

template <typename T, std::size_t N>
struct handler_mock_counter<T, N, std::enable_if_t<!have_handler_mock_n_expectation_setter<T, N>::value>> {
  static constexpr auto value = N;
};

template <typename ExpectationSetter>
struct InLibraryTMRPluginManager final : public TMRPluginManagerBase {
 private:
  static constexpr auto PLUGIN_MOCKED = handler_mock_counter<ExpectationSetter>::value;
  static_assert(PLUGIN_MOCKED != 0, "Need to at least mock one plugin");

  template <std::size_t... Seq>
  static TMTaskHandlerArray_t invoke_expectation_setter_impl(std::index_sequence<Seq...> /*unused*/) noexcept {
    return TMTaskHandlerArray_t{ExpectationSetter::set_expectation(boost::make_shared<HandlerMock<Seq>>())...};
  }

  template <std::size_t Total>
  static TMTaskHandlerArray_t invoke_expectation_setter() noexcept {
    return invoke_expectation_setter_impl(std::make_index_sequence<Total>{});
  }

 public:
  TMTaskHandlerArray_t all_plugins_{invoke_expectation_setter<PLUGIN_MOCKED>()};

  TMTaskHandlerArray_t get_all_plugins() const override { return this->all_plugins_; }
  TMTaskHandler find_task_handler(std::string const &t_input) const override {
    auto const predicate = [&t_input](auto const &t_handler) {
      return t_handler->start_task_handling(t_input) == Decision::Accept;
    };
    auto const matched = std::find_if(this->all_plugins_.begin(), this->all_plugins_.end(), predicate);

    if (matched == this->all_plugins_.end()) {
      return boost::make_shared<ScriptExitHandler>();
    }

    return *matched;
  }
};

}  // namespace tmr_listener

#endif