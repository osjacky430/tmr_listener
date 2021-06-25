#ifndef MOCK_HANDLER_EXPECTATION_HPP_
#define MOCK_HANDLER_EXPECTATION_HPP_

#define MOCK_NUMBER(num) HandlerMock<num>
#define ADD_MOCK_SETTER(mock, name) static void set_impl(mock& name) noexcept

#define MOCK_PTR_NUMBER(num) HandlerMockPtr<num>
#define ADD_MOCK_(mock, name) static mock set_expectation(mock name) noexcept
#define ADD_MOCK(num, name)                \
  ADD_MOCK_SETTER(MOCK_NUMBER(num), name); \
  ADD_MOCK_(MOCK_PTR_NUMBER(num), name) {  \
    set_impl(*name);                       \
    return name;                           \
  }

#define SET_EXPECTATION_(mock, name) void PluginExpectationSetter::set_impl(mock& name) noexcept
#define SET_EXPECTATION(num, name) SET_EXPECTATION_(MOCK_NUMBER(num), name)

namespace tmr_listener {

struct PluginExpectationSetter {
  ADD_MOCK(0, t_first_mock);
};

}  // namespace tmr_listener

#endif