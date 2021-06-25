#ifndef FAKE_SERVER_HPP_
#define FAKE_SERVER_HPP_

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/next_prior.hpp>
#include <boost/thread/scoped_thread.hpp>

#include <chrono>
#include <iostream>

#include "tmr_ext_script/tmr_motion_function.hpp"

namespace tmr_listener {
namespace fake_impl {

class TMRConnection : public boost::enable_shared_from_this<TMRConnection> {
  explicit TMRConnection(boost::asio::io_service& t_io_service) : socket_{t_io_service} {}

  boost::asio::ip::tcp::socket socket_;
  boost::asio::streambuf input_buffer_;

 public:
  using this_ptr = boost::shared_ptr<TMRConnection>;

  static this_ptr create(boost::asio::io_service& t_io_service) { return this_ptr{new TMRConnection(t_io_service)}; }

  boost::asio::ip::tcp::socket& get_socket() noexcept { return this->socket_; }
  boost::asio::streambuf& get_input_buffer() noexcept { return this->input_buffer_; }

  void write(std::string& t_value) { boost::asio::write(this->socket_, boost::asio::buffer(t_value)); }

  void stop() {
    boost::system::error_code ignore_error_code;
    this->socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_send, ignore_error_code);
    boost::asio::read(this->socket_, this->input_buffer_, ignore_error_code);
    this->socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_receive, ignore_error_code);
    this->socket_.close(ignore_error_code);
  }
};

struct wait_response_t {
  std::chrono::milliseconds wait_time_;
};

static const wait_response_t WAIT_FOREVER{std::chrono::milliseconds(0)};

class TMRServer {
  using Content = Expression<std::string>;

  static constexpr auto LISTEN_PORT = 5890;

  std::string write_buffer_;
  TMRConnection::this_ptr connection_;

  bool is_connected_ = false;

  boost::condition_variable connection_signal_;
  boost::mutex connection_mtx_;

  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::endpoint server_{boost::asio::ip::tcp::v4(), LISTEN_PORT};
  boost::asio::ip::tcp::acceptor acceptor_{io_service_};

  boost::asio::io_service::work work_{io_service_};
  boost::scoped_thread<> running_thread_;

  void handle_accept(boost::system::error_code const& t_err, TMRConnection::this_ptr t_connection) noexcept;

  std::string write(std::string, wait_response_t) noexcept;

  void write(std::string) noexcept;

 public:
  TMRServer() = default;

  /**
   * @brief This function writes enter-listen-node message to client and wait for its response for amount of time
   *        specified in t_tag
   *
   * @param t_str Listen node message
   * @param t_tag for tag dispatch
   * @return Response from client
   *
   * @todo Implement timeout, or take timeout variable away, just use it for tag dispatch
   */
  std::string enter_listen_node(std::string t_str, wait_response_t t_tag) noexcept;

  /**
   * @brief This function writes enter-listen-node message to client, this function doesn't wait for client response
   *
   * @param t_str Listen node message
   */
  void enter_listen_node(std::string t_str) noexcept;

  /**
   * @brief This function writes TMSCT OK message to the client
   *
   * @param t_id   ID of the message
   */
  void response_ok_msg(tmr_listener::ID const& t_id) noexcept;

  /**
   * @brief
   */
  void response_tmsta_msg(std::string t_subcmd) noexcept;

  /**
   * @brief This function writes CPERR Error message to the client
   *
   * @param t_err   @ref tmr_listener::ErrorCode
   */
  void send_error(tmr_listener::ErrorCode t_err) noexcept;

  /**
   * @brief This function starts async accept connection request, it runs io_service in the background
   */
  void start();

  /**
   * @brief This function closes the socket, and acceptor. It also stops io service and reset the socket ptr in order
   *        to stop the server fully.
   */
  void stop() noexcept;

  void wait_until_connected() noexcept;

  bool is_connected() const noexcept { return this->is_connected_; }
};

}  // namespace fake_impl
}  // namespace tmr_listener

#endif