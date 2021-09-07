#include "fake_server.hpp"

#include <boost/next_prior.hpp>

using namespace std::string_literals;

namespace {

std::string extract_buffer_data(boost::asio::streambuf& t_buffer, std::size_t t_byte_to_extract) noexcept {
  if (t_byte_to_extract > 0) {
    auto const buffer_begin = boost::asio::buffers_begin(t_buffer.data());
    std::string result{buffer_begin, boost::next(buffer_begin, t_byte_to_extract)};
    t_buffer.consume(t_byte_to_extract);

    return result;
  }

  return std::string{""};
}

}  // namespace

namespace tmr_listener {
namespace fake_impl {

std::string TMRobotServerComm::blocking_read() {
  auto const byte_read = boost::asio::read_until(this->connection_->get_socket(),  //
                                                 this->connection_->get_input_buffer(), "\r\n");
  return ::extract_buffer_data(this->connection_->get_input_buffer(), byte_read);
}

/**
 * @details According to boost asio documentation: A buffer object does not have any ownership of the memory it refers
 *          to. It is the responsibility of the application to ensure the memory region remains valid until it is no
 *          longer required for an I/O operation. When the memory is no longer available, the buffer is said to have
 *          been invalidated. Therefore the value to write is moved to the buffer held by the server.
 */
void TMRobotServerComm::blocking_write(std::string t_str) {
  this->write_buffer_ = std::move(t_str);
  this->connection_->write(this->write_buffer_);
}

void TMRobotServerComm::stop() {
  if (not this->is_connected_) {
    return;
  }

  this->is_connected_ = false;
  this->connection_->stop();
  this->connection_.reset();
  this->acceptor_.close();
  this->io_service_.stop();
}

}  // namespace fake_impl
}  // namespace tmr_listener

namespace tmr_listener {
namespace fake_impl {

using Content = Expression<std::string>;

std::string ListenNodeServer::enter_listen_node(std::string t_str, wait_response_t /*t_tag*/) {
  auto const enter_node_msg = TMSCT << ID{"0"} << Content{std::move(t_str)} << End();
  this->comm_.blocking_write(enter_node_msg->to_str());
  return this->comm_.blocking_read();
}

void ListenNodeServer::enter_listen_node(std::string t_str) {
  auto const enter_node_msg = TMSCT << ID{"0"} << Content{std::move(t_str)} << End();
  this->comm_.blocking_write(enter_node_msg->to_str());
}

/**
 * @todo consider error case
 * @todo Implement abnormal line for t_err = true case, if I find it useful for unit test
 */
void ListenNodeServer::response_ok_msg(tmr_listener::ID const& t_id) {
  auto const response = TMSCT << t_id << Content{"OK"} << End();
  this->comm_.blocking_write(response->to_str());
}

/**
 * @note used to test if the listener handler publish recieved message on recieving TMSTA Subcmd 90 - 99
 */
void ListenNodeServer::send_tmsta_data_msg(int const t_channel, std::string const& t_val) {
  auto const data     = std::to_string(t_channel) + ',' + t_val;
  auto const result   = "$TMSTA," + std::to_string(data.size()) + ',' + data + ',';
  auto const to_write = result + "*" + calculate_checksum(result) + "\r\n";
  this->comm_.blocking_write(to_write);
}

void ListenNodeServer::send_error(tmr_listener::ErrorCode const t_err) {
  using namespace std::string_literals;
  using u_t = std::underlying_type_t<tmr_listener::ErrorCode>;

  auto const msg = (boost::format("$CPERR,2,%02X,") % static_cast<u_t>(t_err)).str();
  this->comm_.blocking_write(msg + '*' + tmr_listener::calculate_checksum(msg) + "\r\n"s);
}

}  // namespace fake_impl
}  // namespace tmr_listener