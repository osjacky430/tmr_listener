#include <boost/range/algorithm_ext.hpp>

#include "tmr_listener/tmr_listener_handle.hpp"

namespace tmr_listener {

Decision ListenerHandle::start_task_handling(std::string const& t_data) noexcept {
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

}  // namespace tmr_listener