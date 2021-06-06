#include <boost/range/algorithm_ext.hpp>

#include "tmr_listener/tmr_listener_handle.hpp"

namespace {

auto get_tokenized_result(std::string const& t_input) noexcept {
  auto const res = boost::tokenizer<boost::char_separator<char>>(t_input, boost::char_separator<char>(";"));
  return std::vector<std::string>(res.begin(), res.end());
}

}  // namespace

namespace tmr_listener {

Decision ListenerHandle::start_task_handling(std::vector<std::string> const& t_data) noexcept {
  auto const ret_val = this->start_task(t_data);
  this->responded_   = ret_val == Decision::Accept ? MessageStatus::Responded : MessageStatus::NotYetRespond;

  return ret_val;
}

/**
 * @details If the command is not empty, then the responsded_ is set to false, indicating that we are waiting for new
 *          responses from the server. Otherwise, the response remains the same.
 */
MessagePtr ListenerHandle::generate_request() noexcept {
  auto const ret_val = this->generate_cmd(this->responded_);

  if (not ret_val->empty()) {
    this->responded_ = MessageStatus::NotYetRespond;
  }

  return ret_val;
}

void ListenerHandle::handle_response(std::vector<std::string> const& t_response) noexcept {
  this->responded_ = MessageStatus::Responded;

  auto const header     = t_response.front();
  auto const data_begin = std::next(t_response.begin(), 2);
  auto const data_end   = std::prev(t_response.end());

  if (header == TMSTA) {
    TMSTAResponse const resp{boost::lexical_cast<int>(*data_begin),
                             std::vector<std::string>(std::next(data_begin), data_end)};
    this->response_msg(resp);
  } else if (header == TMSCT) {
    using namespace boost::range;
    using namespace boost::adaptors;

    auto const result       = get_tokenized_result(*(std::next(data_begin)));
    auto const abnormal_str = std::vector<std::string>{std::next(result.begin()), result.end()};
    auto const line_num     = abnormal_str | transformed(boost::lexical_cast<int, std::string>);

    TMSCTResponse const resp{*data_begin, result[0] == "OK", std::vector<int>{line_num.begin(), line_num.end()}};

    this->response_msg(resp);
  } else if (header == CPERR) {
    auto const err = [](std::string const& t_data) {
      if (t_data == "F1") {
        return ErrorCode::NotInListenNode;
      }

      return static_cast<ErrorCode>(boost::lexical_cast<int>(t_data));
    }(*data_begin);

    CPERRResponse const resp{err};
    this->response_msg(resp);
  }

  this->response_msg();
}

}  // namespace tmr_listener