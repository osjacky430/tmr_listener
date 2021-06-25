#include "fake_server.hpp"

namespace {

std::string extract_buffer_data(boost::asio::streambuf& t_buffer, std::size_t t_byte_to_extract) noexcept {
  if (t_byte_to_extract > 0) {
    auto const buffer_begin = boost::asio::buffers_begin(t_buffer.data());
    std::string const result{buffer_begin, boost::next(buffer_begin, t_byte_to_extract)};
    t_buffer.consume(t_byte_to_extract);

    return result;
  }

  return std::string{""};
}

}  // namespace

namespace tmr_listener {
namespace fake_impl {

std::string TMRServer::write(std::string t_str, wait_response_t) noexcept {
  this->write(std::move(t_str));
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
void TMRServer::write(std::string t_str) noexcept {
  this->write_buffer_ = std::move(t_str);
  this->connection_->write(this->write_buffer_);
}

void TMRServer::handle_accept(boost::system::error_code const& t_err, TMRConnection::this_ptr t_connection) noexcept {
  if (not t_err) {
    boost::unique_lock<boost::mutex> lock{this->connection_mtx_};
    this->is_connected_ = true;
    this->connection_signal_.notify_all();
    this->connection_ = t_connection;
  }
}

std::string TMRServer::enter_listen_node(std::string t_str, wait_response_t t_tag) noexcept {
  auto const enter_node_msg = TMSCT << ID{"0"} << Content{std::move(t_str)} << End();
  return this->write(enter_node_msg->to_str(), t_tag);
}

void TMRServer::enter_listen_node(std::string t_str) noexcept {
  auto const enter_node_msg = TMSCT << ID{"0"} << Content{std::move(t_str)} << End();
  this->write(enter_node_msg->to_str());
}

/**
 * @todo consider error case
 * @todo Implement abnormal line for t_err = true case, if I find it useful for unit test
 */
void TMRServer::response_ok_msg(tmr_listener::ID const& t_id) noexcept {
  auto const response = TMSCT << t_id << Content{"OK"} << End();
  this->write(response->to_str());
}

void TMRServer::send_error(tmr_listener::ErrorCode const t_err) noexcept {
  using namespace std::string_literals;
  using u_t = std::underlying_type_t<tmr_listener::ErrorCode>;

  auto const msg = (boost::format("$CPERR,2,%02X,") % static_cast<u_t>(t_err)).str();
  this->write(msg + '*' + tmr_listener::calculate_checksum(msg) + "\r\n"s);
}

/**
 * @details Current implementation only allow one connection
 * @todo    Perhaps extend to multi connections in the future if we consider accepting multiple handlers
 */
void TMRServer::start() {
  if (not this->running_thread_.joinable()) {
    auto const thread_fn = [this]() {  // only take 1 connection currently
      namespace ph = boost::asio::placeholders;
      this->acceptor_.open(this->server_.protocol());
      this->acceptor_.set_option(boost::asio::socket_base::reuse_address(true));
      this->acceptor_.bind(this->server_);
      this->acceptor_.listen();

      auto new_connection = TMRConnection::create(io_service_);
      this->acceptor_.async_accept(new_connection->get_socket(),
                                   boost::bind(&TMRServer::handle_accept, this, ph::error, new_connection));
      try {
        this->io_service_.run();
      } catch (std::exception& t_e) {
        std::cerr << t_e.what();
      }
    };

    this->running_thread_ = boost::scoped_thread<>{boost::thread(thread_fn)};
  }
}

void TMRServer::stop() noexcept {
  this->is_connected_ = false;
  this->connection_->stop();
  this->connection_.reset();
  this->acceptor_.close();
  this->io_service_.stop();
}

void TMRServer::wait_until_connected() noexcept {
  boost::unique_lock<boost::mutex> lock{this->connection_mtx_};
  auto const connection_success = [this]() { return this->is_connected_; };
  this->connection_signal_.wait(lock, connection_success);
}

}  // namespace fake_impl
}  // namespace tmr_listener